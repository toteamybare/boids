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
    if (option.obstacles === undefined){
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
    } else {
        return runBoidsSimulationWithObstacles(
            initialBoids,
            option.visualRange,
            option.minDistance,
            option.maxSpeed,
            option.weights,
            option.tMax,
            option.dt,
            option.obstacles
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

function isIntersected2D(a, b, c, d) {
    const isCDDivided = ((a.x - b.x) * (c.y - a.y) - (a.y - b.y) * (c.x - a.x))
        * ((a.x - b.x) * (d.y - a.y) - (a.y - b.y) * (d.x - a.x))
        < 0
    const isABDivided = ((c.x - d.x) * (a.y - c.y) - (c.y - d.y) * (a.x - c.x))
        * ((c.x - d.x) * (b.y - c.y) - (c.y - d.y) * (b.x - c.x))
        < 0
    return isCDDivided && isABDivided
}

function avoidObstacles(boid, regionedObstacles, visualRange, factor) {
    let neighborObstarcles = getPossibleNeighbors(boid, regionedObstacles, visualRange, visualRange)
    let neighborObstacleEdges = []
    for (let obstacle of neighborObstarcles) {
        if (Vec.distance(boid.position, obstacle.position) < visualRange) {
            for (let edge of obstacle.edges) {
                if (!neighborObstacleEdges.includes(edge)) {
                    neighborObstacleEdges.push(edge)
                }
            }
        }
    }

    let force = boid.accelaration.genZero()
    let visibleSides = []
    for (let edge of neighborObstacleEdges) {
        const isFacing = edge.norm.dot(boid.position.minus(edge.nodes[0])) > 0
        const isAlreadyViewed = visibleSides.includes(edge.sideId)
        if (isFacing && !isAlreadyViewed) {
            const node0 = edge.nodes[0]
            const node1 = edge.nodes[1]
            let n0IsVisible = true
            let n1IsVisible = true
            for (let otherEdge of neighborObstacleEdges) {
                const n0Intersected = isIntersected2D(boid.position, node0, otherEdge.nodes[0], otherEdge.nodes[1])
                const n1Intersected = isIntersected2D(boid.position, node1, otherEdge.nodes[0], otherEdge.nodes[1])
                n0IsVisible = n0IsVisible && !n0Intersected
                n1IsVisible = n1IsVisible && !n1Intersected
            }
            if (n0IsVisible && n1IsVisible) {
                force = force.add(edge.norm)
                visibleSides.push(edge.sideId)
            }
        }
    }

    boid.accelaration = boid.accelaration.add(force.scalarMul(factor))
}

function cellIntersectedNodesOfEdge2D(node1, node2, cellSize) {
    const [x1, y1] = regionHashForVec2(cellSize, cellSize, node1)
    const [x2, y2] = regionHashForVec2(cellSize, cellSize, node2)
    let intersections = []

    for (let xi = Math.min(x1, x2)+1; xi <= Math.max(x1, x2); xi++) {
        if (xi*cellSize !== node1.x && xi*cellSize !== node2.x) {
            const x = xi*cellSize
            const y = node1.y + (node2.y - node1.y) * (xi*cellSize - node1.x) / (node2.x - node1.x)
            let isExists = false
            for (let intersection of intersections) {
                const isEqual = intersection.x === x && intersection.y === y
                isExists = isExists || isEqual
            }
            if (!isExists) {
                intersections.push(new Vec2(x, y))
            }
        }
    }
    for (let yi = Math.min(y1, y2) + 1; yi <= Math.max(y1, y2); yi++) {
        if (yi*cellSize !== node1.y && yi*cellSize !== node2.y) {
            const x = node1.x + (node2.x - node1.x) * (yi*cellSize - node1.y) / (node2.y - node1.y)
            const y = yi*cellSize
            let isExists = false
            for (let intersection of intersections) {
                const isEqual = intersection.x === x && intersection.y === y
                isExists = isExists || isEqual
            }
            intersections.push(new Vec2(x, y))
        }
    }

    return intersections.sort((a, b) => a.minus(node1).sqrNorm() - b.minus(node1).sqrNorm())
}

function networkOfPolygonObstacle2D(nodes, isCCW, isNormOut, cellSize, minNodeId, minEdgeId, minSideId) {
    let network = {
        nodes: [],
        edges: []
    }
    let startNode = null
    let endNode = null
    for (let i=0; i<nodes.length; i++) {
        startNode = nodes[i]
        if (i < nodes.length-1) {
            endNode = nodes[i+1]
        } else {
            endNode = nodes[0]
        }
        let addedNodes = [startNode]
        addedNodes = addedNodes.concat(cellIntersectedNodesOfEdge2D(startNode, endNode, cellSize))

        let vec1 = null
        let vec2 = null
        for (let j=0; j<addedNodes.length; j++) {
            vec1 = addedNodes[j]
            if (j < addedNodes.length - 1) {
                vec2 = addedNodes[j+1]
            } else {
                vec2 = endNode
            }
            const vec = vec2.minus(vec1)
            network.edges.push({
                nodes: [vec1, vec2],
                norm: !(isCCW && isNormOut)? new Vec2(-vec.y, vec.x).normalize(): new Vec2(vec.y, -vec.x).normalize(),
                sideId: minSideId + i
            })
            network.nodes.push({
                position: vec1
            })
        }
    }
    
    for (let i = 0; i < network.edges.length; i++) {
        network.edges[i].id = minEdgeId + i
    }
    
    for (let i = 0; i < network.nodes.length; i++) {
        network.nodes[i].id = minNodeId + i
        if (i !== 0) {
            network.nodes[i].edges = [ network.edges[i-1], network.edges[i]]
        } else {
            network.nodes[i].edges = [ network.edges[network.edges.length - 1], network.edges[0]]
        }
    }

    return network
}

function polygonsToRegionedNodes (polygons, cellSize) {
    let network = {nodes: [], edges: []}
    let numSides = 0
    for (let polygon of polygons) {
        const {nodes, isCCW, isNormOut} = polygon
        const addedNetwork = networkOfPolygonObstacle2D(nodes, isCCW, isNormOut, cellSize, network.nodes.length, network.edges.length, numSides)
        network.nodes = network.nodes.concat(addedNetwork.nodes)
        network.edges = network.edges.concat(addedNetwork.edges)
        numSides = numSides + nodes.length
    }
    return boidsToRegions(network.nodes, cellSize, cellSize)
}

function runBoidsSimulationWithObstacles(initialBoids, visualRange, minDistance, maxSpeed, weights, tMax, dt, polygons) {
    let history = [];
    let boids = initialBoids;
    const cellSize = Math.max(visualRange, minDistance)
    const regionedObstacles = polygonsToRegionedNodes(polygons, visualRange)
    const record0 = cloneBoids(boids)
    history.push({t:0, boids: record0, regions: boidsToRegions(record0, cellSize, cellSize)});

    for (let t = 1; t <= tMax; t++) {
        let lastStates = history.slice(-1)[0]
        let last = lastStates.boids
        let regions = lastStates.regions

        for (let boid of boids) {
            const possibleNeighbors = getPossibleNeighbors(boid, regions, cellSize, cellSize)
            boid.accelaration = boid.accelaration.genZero()

            flyToCenter(boid, possibleNeighbors, visualRange, weights.flyToCenter)
            avoidOthers(boid, possibleNeighbors, minDistance, weights.avoidOthers)
            matchVelocity(boid, possibleNeighbors, visualRange, weights.matchVelocity)
            avoidObstacles(boid, regionedObstacles, visualRange, weights.avoidObstacles)

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