import { download } from "./util.js"
import { runSim as runSimControl} from "./simulator_control.js"
import { runSim as runSimControlWithBucket } from "./simulator_control_bucket.js"
import { runSim as runSimMonoid} from "./simulator_monoid.js"
import { runSim as runSimMonoidWithBucket } from "./simulator_monoid_bucket.js"
import { runSim as runSimMonoidWithObstacles } from "./simulator_monoid_obstacles.js"
import { Vec, Vec2} from "./vector.js"

function convertFormat (stateSeries) {
    return stateSeries.map(state => {
        return {
            t: state.t,
            state: state.boids.map(boid => {
                return {
                    x: boid.position.x,
                    y: boid.position.y,
                    vx: boid.velocity.x,
                    vy: boid.velocity.y
                }
            })
        }
    })
}

function randomBoids(numBoids, width, height) {
    return Array(numBoids).fill(undefined).map((_, i) => {
        return {
            position: new Vec2(Math.random() * width, Math.random() * height),
            velocity: new Vec2(Math.random() * 2 - 1, Math.random() * 2 - 1),
            accelaration: Vec2.zero(),
            id: i
    }})
}

function hasSamePVA(a, b) {
    function toVec2(obj) {
        return new Vec2(obj.x, obj.y)
    }
    return toVec2(a.position).equals(b.position)
        && toVec2(a.velocity).equals(b.velocity)
        && toVec2(a.accelaration).equals(b.accelaration)
}

function cloneBoids (boids) {
    return boids.map ((boid, i) => {
        return {
            position: boid.position.clone(),
            velocity: boid.velocity.clone(),
            accelaration: boid.velocity.clone(),
            id: i
        }
    })
}

function sum(arr) {
    return arr.reduce((accum, num) => accum + num, 0)
}

function avg(arr) {
    return arr.length? sum(arr)/arr.length: 0
}

function stats(arr) {
    const average = avg(arr)
    const variance = avg(arr.map(val => (val - average)*(val - average)))
    const sd = Math.sqrt(variance)
    return [average, sd]
}

function speedTest(pre, func, times) {
    const result = []
    for (let i =0; i< times; i++) {
        pre()
        const start = performance.now()
        func()
        const end = performance.now()
        result.push(end - start)
    }
    return stats(result)
}

function randomInt (max) {
    return Math.floor(Math.random() * (max + 1))
}

function equalityVerification(b1, b2) {
    let isEqual = true
    let numBoids = b1[0].boids.length
    b1.forEach((atI, i) => {
        const sampleIndex = randomInt(numBoids - 1)
        const isEqualAtI = hasSamePVA(atI.boids[sampleIndex], b2[i].boids[sampleIndex])
        console.log(isEqualAtI)
        isEqual = isEqual && isEqualAtI
    })
    return isEqual
}

const range = (n, start=0) => Array.from({length: n}, (_, i) => start + i)

window.onload = () => {
    let boids = []
    const option = {
        n: 500,
        visualRange: 75, 
        minDistance: 20,
        margin: 200,
        width: 1000,
        height: 1000,
        maxSpeed: 15,
        tMax: 500,
        dt: 1,
        weights: {
            flyToCenter: 0.005,
            avoidOthers: 0.05,
            matchVelocity: 0.1,
            keepWithinBound: 1,
            avoidObstacles: 2
        },
        useBucket: true,
        obstacles: [
            // {
            //     nodes: [new Vec2(0, 0), new Vec2(1000, 0), new Vec2(1000, 1000), new Vec2(0, 1000)],
            //     isCCW: true,
            //     isNormOut: false
            // }, 
            // {
            //     nodes: [new Vec2(400, 400), new Vec2(600, 400), new Vec2(600, 600), new Vec2(400, 600)],
            //     isCCW: true,
            //     isNormOut: true
            // },
            {
                nodes: range(12).map(n => {
                    return new Vec2(Math.cos(2 * Math.PI / 12 * n) * 400 + 500, Math.sin(2 * Math.PI / 12 * n) * 400 + 500)
                }),
                isCCW: true,
                isNormOut: false
            }
        ]
    }

    document.getElementById("generateBoidsBtn").addEventListener("click", () => {
        boids = randomBoids(option.n, option.width, option.height)
        console.log(boids)
    })

    document.getElementById("simControlBtn").addEventListener("click", () => {
        download( convertFormat(runSimControl(cloneBoids(boids), option)) )
    })

    document.getElementById("simConstrolBucketBtn").addEventListener("click", () => {
        download( convertFormat(runSimControlWithBucket(cloneBoids(boids), option)) )
    })

    document.getElementById("simMonoidBtn").addEventListener("click", () => {
        download( convertFormat(runSimMonoid(cloneBoids(boids), option)) )
    })

    document.getElementById("simMonoidBucketBtn").addEventListener("click", () => {
        download( convertFormat(runSimMonoidWithBucket(cloneBoids(boids), option)) )
    })

    document.getElementById("simMonoidObstaclesBtn").addEventListener("click", () => {
        download( convertFormat(runSimMonoidWithObstacles(cloneBoids(boids), option)) )
    })
    

    document.getElementById("verifyBtn").addEventListener("click", () => {
        boids = randomBoids(option.n, option.width, option.height)
        // const control = runSimControl(cloneBoids(boids), option)
        // console.log(control)
        const controlB = runSimControlWithBucket(cloneBoids(boids), option)
        console.log(controlB)
        // const monoid = runSimMonoid(cloneBoids(boids), option)
        // console.log(monoid)
        const monoidB = runSimMonoidWithBucket(cloneBoids(boids), option)
        console.log(monoidB)

        console.log(equalityVerification(controlB, controlB))
        // console.log(equalityVerification(monoidB, monoid))
    })

    document.getElementById("speedTestControlBtn").addEventListener("click", () => {
        const result = speedTest(()=> {
            boids = randomBoids(option.n, option.width, option.height)
        }, () => {
            runSimControl(boids, option)
        }, 10)
        console.log(result)
    })
    document.getElementById("speedTestControlBucketBtn").addEventListener("click", () => {
        const result = speedTest(()=> {
            boids = randomBoids(option.n, option.width, option.height)
        }, () => {
            runSimControlWithBucket(boids, option)
        }, 10)
        console.log(result)
    })
    document.getElementById("speedTestMonoidBtn").addEventListener("click", () => {
        const result = speedTest(()=> {
            boids = randomBoids(option.n, option.width, option.height)
        }, () => {
            runSimMonoid(boids, option)
        }, 10)
        console.log(result)
    })
    document.getElementById("speedTestMonoidBucketBtn").addEventListener("click", () => {
        const result = speedTest(()=> {
            boids = randomBoids(option.n, option.width, option.height)
        }, () => {
            runSimMonoidWithBucket(boids, option)
        }, 10)
        console.log(result)
    })
    document.getElementById("speedTestMonoidObstaclesBtn").addEventListener("click", () => {
        const result = speedTest(()=> {
            boids = randomBoids(option.n, option.width, option.height)
        }, () => {
            runSimMonoidWithObstacles(boids, option)
        }, 10)
        console.log(result)
    })

    document.getElementById("test").addEventListener("click", () => {
        // const edges = []
        // const nodes = []

        // function edgeNorm(n1, n2, isClockWise) {
        //     const edgeVec = n2.minus(n1)
        //     return isClockWise? new Vec2(edgeVec.y, - edgeVec.x).normalize(): new Vec2(- edgeVec.y, edgeVec.x).normalize()
        // }

        // function addEdge(n1, n2, normIsClockWise) {
        //     const edgeId = edges.length
        //     const node1Id = nodes.length
        //     const node2Id = node1Id + 1
        //     const edge = {
        //         nodes: [
        //             n1, n2
        //         ],
        //         id: edgeId,
        //         norm: edgeNorm(n1, n2, normIsClockWise)
        //     }
        //     const node1 = {
        //         position: n1,
        //         edges: [edge],
        //         id: node1Id
        //     }
        //     const node2 = {
        //         position: n1,
        //         edges: [edge],
        //         id: node2Id
        //     }
        //     nodes.push(node1)
        //     nodes.push(node2)
        //     edges.push(edge)
        // }

        // addEdge(new Vec2(0, 0), new Vec2(1, 2), true)
        // addEdge(new Vec2(-1, 2), new Vec2(0, 3))

        const regionedObstacles = polygonsToRegionedNodes([
            {
                nodes: [new Vec2(0, 0), new Vec2(20, 0), new Vec2(0, 20)],
                isCCW: true,
                isNormOut: true
            }
        ], 10)
    })
}
