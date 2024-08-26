function initBoids(numBoids, width, height) {
    let boids = [];
    for (var i = 0; i < numBoids; i += 1) {
        boids[i] = {
            x: Math.random() * width,
            y: Math.random() * height,
            vx: Math.random() * 2 - 1,
            vy: Math.random() * 2 - 1,
        };
    }
    return boids
}

function distance(boid1, boid2) {
    return Math.sqrt(
        (boid1.x - boid2.x) * (boid1.x - boid2.x) +
        (boid1.y - boid2.y) * (boid1.y - boid2.y),
    );
}

function keepWithinBounds(boid, width, height) {
    const margin = 200;
    const turnFactor = 1;
    if (boid.x < margin) {
        boid.vx += turnFactor;
    }
    if (boid.x > width - margin) {
        boid.vx -= turnFactor
    }
    if (boid.y < margin) {
        boid.vy += turnFactor;
    }
    if (boid.y > height - margin) {
        boid.vy -= turnFactor;
    }
}

function flyTowardsCenter(boid, boids, visualRange) {
    const centeringFactor = 0.005;
    let centerX = 0;
    let centerY = 0;
    let numNeighbors = 0;
    for (let otherBoid of boids) {
        if (distance(boid, otherBoid) < visualRange) {
            centerX += otherBoid.x;
            centerY += otherBoid.y;
            numNeighbors += 1;
        }
    }
    if (numNeighbors) {
        centerX = centerX / numNeighbors;
        centerY = centerY / numNeighbors;
        boid.vx += (centerX - boid.x) * centeringFactor;
        boid.vy += (centerY - boid.y) * centeringFactor;
    }
}

function avoidOthers(boid, boids, minDistance) {
    const avoidFactor = 0.05;
    let moveX = 0;
    let moveY = 0;
    for (let otherBoid of boids) {
        if (otherBoid !== boid) {
            if (distance(boid, otherBoid) < minDistance) {
                moveX += boid.x - otherBoid.x;
                moveY += boid.y - otherBoid.y;
            }
        }
    }
    boid.vx += moveX * avoidFactor;
    boid.vy += moveY * avoidFactor;
}

function matchVelocity(boid, boids, visualRange) {
    const matchingFactor = 0.05;
    let avgDX = 0;
    let avgDY = 0;
    let numNeighbors = 0;
    for (let otherBoid of boids) {
        if (distance(boid, otherBoid) < visualRange) {
            avgDX += otherBoid.vx;
            avgDY += otherBoid.vy;
            numNeighbors += 1;
        }
    }
    if (numNeighbors) {
        avgDX = avgDX / numNeighbors;
        avgDY = avgDY / numNeighbors;
        boid.vx += avgDX * matchingFactor;
        boid.vy += avgDY * matchingFactor;
    }
}

function limitSpeed(boid) {
    const speedLimit = 15;
    const speed = Math.sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    if (speed > speedLimit) {
        boid.vx = (boid.vx / speed) * speedLimit;
        boid.vy = (boid.vy / speed) * speedLimit;
    }
}

export const runSim = function run() {
    let width = 1000;
    let height = 1000;
    const numBoids = 1000;
    let history = [];
    let boids = initBoids(numBoids, width, height);
    history.push({t:0, state: JSON.parse(JSON.stringify(boids))});
    let tMax = 1000;
    const start = performance.now();
    for (let t = 1; t<tMax; t++) {
        let last = history.slice(-1)[0].state;
        for (let boid of boids) {
            flyTowardsCenter(boid, last, 75);
            avoidOthers(boid, last, 20);
            matchVelocity(boid, last, 75);
            limitSpeed(boid);
            keepWithinBounds(boid, width, height);
            boid.x += boid.vx;
            boid.y += boid.vy;
        }
        history.push({t:t, state: JSON.parse(JSON.stringify(boids))});
    }
    const end = performance.now();
    console.log(end-start);
    console.log(history);
    return history;
}