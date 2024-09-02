import {Vec, Vec2} from "./vector.js"

function keepWithinBounds(boid, width, height, margin, factor) {
    if (boid.position.x < margin) {
        boid.accelaration.x += factor;
    }
    if (boid.position.x > width - margin) {
        boid.accelaration.x -= factor
    }
    if (boid.position.y < margin) {
        boid.accelaration.y += factor;
    }
    if (boid.position.y > height - margin) {
        boid.accelaration.y -= factor;
    }
}

function flyToCenter(boid, boids, visualRange, factor) {
    let center = boid.position.genZero()
    let numNeighbors = 0;
    for (let otherBoid of boids) {
        if (boid.id !== otherBoid.id) {
            if (Vec.distance(boid.position, otherBoid.position) < visualRange) {
                center = center.add(otherBoid.position)
                numNeighbors += 1
            }
        }
    }
    if (numNeighbors) {
        center = center.scalarMul(1/numNeighbors).minus(boid.position)
        boid.accelaration = boid.accelaration.add(center.scalarMul(factor));
    }
}

function avoidOthers(boid, boids, minDistance, factor) {
    let center = boid.position.genZero()
    let numNeighbors = 0
    for (let otherBoid of boids) {
        if (boid.id !== otherBoid.id) {
            if (Vec.distance(boid.position, otherBoid.position) < minDistance) {
                center = center.add(otherBoid.position)
                numNeighbors += 1
            }
        }
    }
    if (numNeighbors) {
        center = boid.position.minus(center.scalarMul(1/numNeighbors)).scalarMul(numNeighbors)
        boid.accelaration = boid.accelaration.add(center.scalarMul(factor))
    }
}

function matchVelocity(boid, boids, visualRange, factor) {
    let avgVel = boid.velocity.genZero()
    let numNeighbors = 0;
    for (let otherBoid of boids) {
        if (boid.id !== otherBoid.id) {
            if (Vec.distance(boid.position, otherBoid.position) < visualRange) {
                avgVel = avgVel.add(otherBoid.velocity)
                numNeighbors += 1;
            }
        }
    }
    if (numNeighbors) {
        avgVel = avgVel.scalarMul(1/numNeighbors)
        boid.accelaration = boid.accelaration.add(avgVel.scalarMul(factor))
    }
}

function limitSpeed(boid, maxSpeed) {
    const speed = boid.velocity.norm();
    if (speed > maxSpeed) {
        boid.velocity = boid.velocity.scalarMul(maxSpeed/speed)
    }
}

function cloneBoids (boids) {
    const clone = []
    for (let boid of boids) {
        clone.push({
            position: boid.position.clone(),
            velocity: boid.velocity.clone(),
            accelaration: boid.accelaration.clone(),
            id: boid.id
        })
    }
    return clone
}

function runBoidsSimulation(initialBoids, visualRange, minDistance, margin, width, height, maxSpeed, weights, tMax, dt) {
    let history = [];
    let boids = initialBoids;
    history.push({t:0, boids: cloneBoids(boids)});

    for (let t = 1; t <= tMax; t++) {
        let last = history.slice(-1)[0].boids

        for (let boid of boids) {
            boid.accelaration = boid.accelaration.genZero()

            flyToCenter(boid, last, visualRange, weights.flyToCenter)
            avoidOthers(boid, last, minDistance, weights.avoidOthers)
            matchVelocity(boid, last, visualRange, weights.matchVelocity)
            keepWithinBounds(boid, width, height, margin, weights.keepWithinBound)

            boid.velocity = boid.velocity.add(boid.accelaration.scalarMul(dt))
            limitSpeed(boid, maxSpeed);
            boid.position = boid.position.add(boid.velocity.scalarMul(dt))
            boid.accelaration = dt? boid.velocity.minus(last[boid.id].velocity).scalarMul(1/dt): boid.accelaration.genZero()
        }

        history.push({t:t, boids: cloneBoids(boids)});
    }

    return history;
}

export const runSim = function run(initialBoids, option) {
    if (!option.useBucket){
        return runBoidsSimulation(
            initialBoids,
            option.visualRange,
            option.minDistance,
            option.margin,
            option.width,
            option.height,
            option.maxSpeed,
            option.weights,
            option.tMax,
            option.dt
        )
    } else {
        return runBoidsSimulationWithBucket(
            initialBoids,
            option.visualRange,
            option.minDistance,
            option.margin,
            option.width,
            option.height,
            option.maxSpeed,
            option.weights,
            option.tMax,
            option.dt
        )
    }
}

function regionHashForVec2(cellWidth, cellHeight, vec) {
    const xi = Math.floor(vec.x/cellWidth)
    const yi = Math.floor(vec.y/cellHeight)
    return [xi, yi]
}

function boidsToRegions(boids, cellWidth, cellHeight) {
    let regions = {}

    boids.forEach((boid) => {
        const [xi, yi] = regionHashForVec2(cellWidth, cellHeight, boid.position)
        if (regions[xi] === undefined) {
            regions[xi] = {}
        }
        if (regions[xi][yi] === undefined){
            regions[xi][yi] = []
        }
        regions[xi][yi].push(boid)
    })

    return regions
}

function eightNeighbors (x, y) {
    return [[x-1, y-1], [x, y-1], [x+1, y-1], 
        [x-1, y], [x, y], [x+1, y], 
        [x-1, y+1], [x, y+1], [x+1, y+1]
    ]
}

function getPossibleNeighbors (boid, regions, cellWidth, cellHeight) {
    const [xi, yi] = regionHashForVec2(cellWidth, cellHeight, boid.position)
    let neighborBoids = []
    eightNeighbors(xi, yi).forEach((cell) => {
        const regionsX = regions[cell[0]]
        if (regionsX !== undefined){
            const region = regionsX[cell[1]]
            if (region !== undefined) {
                neighborBoids = neighborBoids.concat(region)
            }
        }
    })
    return neighborBoids
}

function runBoidsSimulationWithBucket(initialBoids, visualRange, minDistance, margin, width, height, maxSpeed, weights, tMax, dt) {
    let history = [];
    let boids = initialBoids;
    const cellSize = Math.max(visualRange, minDistance)
    const record0 = cloneBoids(boids)
    history.push({t:0, boids: record0, regions: boidsToRegions(record0, cellSize, cellSize)});

    for (let t = 1; t <= tMax; t++) {
        let lastStates = history.slice(-1)[0]
        let last = lastStates.boids
        let regions = lastStates.regions

        for (let boid of boids) {
            const possibleNeighbors = getPossibleNeighbors(boid, regions, cellSize, cellSize)
            boid.accelaration = boid.accelaration.genZero()

            flyToCenter(boid, possibleNeighbors, visualRange, weights.flyToCenter);
            avoidOthers(boid, possibleNeighbors, minDistance, weights.avoidOthers);
            matchVelocity(boid, possibleNeighbors, visualRange, weights.matchVelocity);
            keepWithinBounds(boid, width, height, margin, weights.keepWithinBound);

            boid.velocity = boid.velocity.add(boid.accelaration.scalarMul(dt))
            limitSpeed(boid, maxSpeed);
            boid.position = boid.position.add(boid.velocity.scalarMul(dt))
            boid.accelaration = dt? boid.velocity.minus(last[boid.id].velocity).scalarMul(1/dt): boid.accelaration.genZero()
        }

        const record = cloneBoids(boids)
        history.push({t:t, boids: record, regions: boidsToRegions(record, cellSize, cellSize)});
    }
    return history;
}