import {Vec, Vec2} from "./vector.js"

function S_timeSystem(current, elapsed) {
    return current + elapsed.num
}

function SG_accelaratablePoint (maxSpeed = undefined) {
    return  (state, inputs) => {
        const newStates = inputs.list.reduce((accum, input) => {
            const [time, acc] = input
            const tentativeVelocity = accum.velocity.add(acc.scalarMul(time))
            const speed = tentativeVelocity.norm()
            const velocity = (maxSpeed != undefined && speed > maxSpeed)? tentativeVelocity.scalarMul(maxSpeed/speed): tentativeVelocity
            return {
                velocity: velocity,
                position: accum.position.add(velocity.scalarMul(time)),
                sumTime: accum.sumTime + time
            }
        }, {position: state.position, velocity: state.velocity, sumTime: 0})
        const newAcc = newStates.sumTime? newStates.velocity.minus(state.velocity).scalarMul(1/newStates.sumTime): state.accelaration.genZero()
        return {position: newStates.position, velocity: newStates.velocity, accelaration: newAcc}
    }
}

function P_identity(_, input) {
    return input
}

function P_identityAndState(state, input) {
    return new MList([[input.getValue(), state]])
}

function flyToCenterForce(me, others) {
    return others.length? Vec.ave(mapWithKey(others, "position")).minus(me.position) : me.accelaration.genZero()
}

function avoidOthersForce(me, others) {
    return others.length? me.position.minus( Vec.ave(mapWithKey(others, "position")) ).scalarMul(others.length) : me.accelaration.genZero()
}

function matchVelocityForce(me, others) {
    return others.length? Vec.ave(mapWithKey(others, "velocity")) : me.accelaration.genZero()
}

function squarePotential(pos, x0, y0, x1, y1) {
    const force = Vec2.zero()
    if (pos.x < x0) force.x = 1
    if (pos.x > x1) force.x = -1
    if (pos.y < y0) force.y = 1
    if (pos.y > y1) force.y = -1
    return force
}

function PG_boidForce(visualRange, minDistance, margin, width, height, weights) {
    return (myState, inputs) => {
        return inputs.list.reduce((accum, current) => {
            const [time, others] = current
            const viewedBoid = others.filter(other => Vec.distance(other.position, myState.position) < visualRange && myState.id !== other.id)
            const collidingBoid = others.filter(other => Vec.distance(other.position, myState.position) < minDistance && myState.id !== other.id)
            const zero = myState.accelaration.genZero()
            const force = Vec.sum([
                weights.flyToCenter !==0 ? flyToCenterForce(myState, viewedBoid).scalarMul(weights.flyToCenter): zero, 
                weights.avoidOthers !==0 ? avoidOthersForce(myState, collidingBoid).scalarMul(weights.avoidOthers): zero,
                weights.matchVelocity !==0 ? matchVelocityForce(myState, collidingBoid).scalarMul(weights.matchVelocity): zero,
                weights.keepWithinBound !==0 ? squarePotential(myState.position, margin, margin, width-margin, height-margin).scalarMul(weights.keepWithinBound): zero
            ])
            return accum.add(new MList([[time, force]]))
        }, MList.zero())
    }
}

function SG_boid (visualRange, minDistance, margin, width, height, maxSpeed, weights) {
    return (myState, inputs) => {
        const accelarations = PG_boidForce(visualRange, minDistance, margin, width, height, weights)(myState, inputs)
        return {...SG_accelaratablePoint(maxSpeed)(myState, accelarations), id: myState.id}
    }
}

const SG_boids = (n, visualRange, minDistance, margin, width, height, maxSpeed, weights) => composeSystemsAsArray (
    Array(n).fill(SG_boid(visualRange, minDistance, margin, width, height, maxSpeed, weights)),
    Array(n).fill(P_identityAndState)
)

const SG_boidsWT = (n, visualRange, minDistance, margin, width, height, maxSpeed, weights) => composeSystemsAsObject(
    ["t", "boids"],
    [S_timeSystem, SG_boids(n, visualRange, minDistance, margin, width, height, maxSpeed, weights)],
    [P_identity, P_identity]
)

function composeSystemsAsArray(transitions, propagations) {
    const projections = projectionFunctions(transitions.length)
    return (state, input) => {
        return transitions.map((transition, i) => {
            return transition(projections[i](state), propagations[i](state, input))
        })
    }
}

function composeSystemsAsObject(keys, transitions, propagations) {
    return (state, input) => {
        return keys.reduce((accum, key, i) => {
            accum[key] = transitions[i](state[key], propagations[i](state, input))
            return accum
        }, {})
    }
}

function selfFeedingSimulation (transition, inputGenerator, toContinue, state0, Minput) {
    function sim(transition, inputGenerator, toContinue, states, inputs, sumInput) {
        const newInput = inputGenerator(states, inputs)
        const newState = transition(states.at(-1), newInput)
        const newSumInput = sumInput.add(newInput)
        if (toContinue(newState, newSumInput)) {
            return sim(transition, inputGenerator, toContinue, [...states, newState], [...inputs, newInput], newSumInput)
        } else  {
            return {
                states: states,
                inputs: inputs,
                sumInput: sumInput
            };
        }
    }
    return sim(transition, inputGenerator, toContinue, [state0], [Minput.zero()], Minput.zero())
}

class MNum {
    constructor(num) {
        this.num = num
    }
    add(second) {
        return new MNum(this.num + second.num)
    }
    getValue() {
        return this.num
    }
    static zero() {
        return new MNum(0)
    }
}

class MList {
    constructor(list) {
        this.list = list
    }
    add (second) {
        return new MList([...this.list, ...second.list])
    }
    getValue() {
        return this.list
    }
    map(func) {
        return new MList(this.list.map(func))
    }
    static zero() {
        return new MList([])
    }
}

function constant(value){
    return () => value
}

function mapWithKey(array, key) {
    return array.map(obj => obj[key])
}

function projectionFunctions(length) {
    return Array.from(Array(length)).map((_, i)=> {
        return (arr) => arr[i]
    })
}

function timeLimit(tmax) {
    return (state, _) => {
        return state.t <= tmax
    }
}

export function runSim(initialBoids, option){
    if (option.obstacles === undefined) {
        if (!option.useBucket) {
            return selfFeedingSimulation(
                SG_boidsWT(initialBoids.length,
                    option.visualRange,
                    option.minDistance,
                    option.margin,
                    option.width,
                    option.height,
                    option.maxSpeed,
                    option.weights),
                constant(new MNum(option.dt)),
                timeLimit(option.tMax),
                {t: 0, boids: initialBoids},
                MNum
            ).states
        } else {
            const cellSize = Math.max(option.visualRange, option.minDistance)
            return selfFeedingSimulation(
                SG_boidsWithBucket(option.visualRange,
                    option.minDistance,
                    option.margin,
                    option.width,
                    option.height,
                    option.maxSpeed,
                    option.weights),
                constant(new MNum(option.dt)),
                timeLimit(option.tMax),
                {t: 0, boids: initialBoids, regions: positionedObjectsToRegions(initialBoids, cellSize, cellSize)},
                MNum
            ).states
        }
    } else {
        const cellSize = Math.max(option.visualRange, option.minDistance)
        return selfFeedingSimulation(
            SG_boidsWithStaticObs(
                option.visualRange,
                option.minDistance,
                option.maxSpeed,
                option.weights,
                option.obstacles),
            constant(new MNum(option.dt)),
            timeLimit(option.tMax),
            {t: 0, boids: initialBoids, regions: positionedObjectsToRegions(initialBoids, cellSize, cellSize)},
            MNum
        ).states
    }
}

function regionHash(cellSize) {
    return vec => vec.map(c => Math.floor(c/cellSize))
}

function positionedObjectsToRegions(objs, cellSize) {
    const hash = regionHash(cellSize)
    return objs.reduce((regions, obj) => {
        const {x, y} = hash(obj.position)
        if (regions[x] === undefined) {
            regions[x] = {}
        }
        if (regions[x][y] === undefined){
            regions[x][y] = []
        }
        regions[x][y].push(obj)
        return regions
    }, {})
}

function eightNeighbors (x, y) {
    return [[x-1, y-1], [x, y-1], [x+1, y-1], 
        [x-1, y], [x, y], [x+1, y], 
        [x-1, y+1], [x, y+1], [x+1, y+1]
    ]
}

function getPossibleNeighbors (boid, regions, cellSize) {
    const {x, y} = regionHash(cellSize)(boid.position)
    return eightNeighbors(x, y).map((hash) => {
        return (regions[hash[0]] && regions[hash[0]][hash[1]])? regions[hash[0]][hash[1]]: []
    }).flat()
}

function SG_boidsWithBucket (visualRange, minDistance, margin, width, height, maxSpeed, weights) {
    const boidSystem = SG_boid(visualRange, minDistance, margin, width, height, maxSpeed, weights)
    return SG_boidsAndRegions(Math.max(visualRange, minDistance), boidSystem)
}

function SG_boidsAndRegions (cellSize, S_boid) {
    return (state, input) => {
        const boids = state.boids.map((boid) => {
            const neighborBoids = getPossibleNeighbors(boid, state.regions, cellSize)
            return S_boid(boid, new MList([[input.getValue(), neighborBoids]]))
        })
        const regions = positionedObjectsToRegions(boids, cellSize)
        return {
            t: S_timeSystem(state.t, input),
            boids: boids,
            regions: regions
        }
    }
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

function visibleEdges (position, neighborNodes) {
    const candidates = neighborNodes.reduce((edges, node) => {
        node.edges.forEach(edge => edges.add(edge))
        return edges
    }, new Set())
    return [...candidates].filter(edge => {
        const isFacing = edge.norm.dot(position.minus(edge.nodes[0])) > 0
        const nodeIsVisible = (n) => [...candidates].reduce((isVisible, otherEdge) => {
            return isVisible
                && ((edge.id === otherEdge.id) || !isIntersected2D(position, n, otherEdge.nodes[0], otherEdge.nodes[1]))
        }, true)
        return isFacing && (nodeIsVisible(edge.nodes[0]) || nodeIsVisible(edge.nodes[1]))
    })
}

const range = (n, start=0) => Array.from({length: n}, (_, i) => start + i)

function concatWithNoEquality (arr1, arr2, equals) {
    return arr1.concat(arr2.filter( (elem2) => arr1.reduce((flag, elem1) => flag && !equals(elem1, elem2), true) ))
}

function cellIntersectedNodesOfEdge2D(node1, node2, cellSize) {
    const hash = regionHash(cellSize)
    const {x:x1, y:y1} = hash(node1)
    const {x:x2, y:y2} = hash(node2)
    const xIntersections = range(Math.abs(x1 - x2), Math.min(x1, x2) + 1).map((x) => {
        return (x*cellSize !== node1.x && x*cellSize !== node2.x)
            ? new Vec2(x*cellSize, node1.y + (node2.y - node1.y) * (x*cellSize - node1.x) / (node2.x - node1.x))
            : undefined
    }).filter(x=> x!== undefined)
    const yIntersections = range(Math.abs(y1 - y2), Math.min(y1, y2) + 1).map((y) => {
        return (y*cellSize !== node1.y && y*cellSize !== node2.y)
            ? new Vec2(node1.x + (node2.x - node1.x) * (y*cellSize - node1.y) / (node2.y - node1.y), y*cellSize)
            : undefined
    }).filter(x=> x!== undefined)
    return concatWithNoEquality(xIntersections, yIntersections, (a,b) => a.equals(b))
        .sort((a,b) => a.minus(node1).sqrNorm() - b.minus(node1).sqrNorm())
}

function networkOfPolygonObstacle2D(nodes, isCCW, isNormOut, cellSize, minNodeId, minEdgeId) {
    const network = nodes.reduce((accum, startNode, i) => {
        const endNode = nodes[i+1] !== undefined? nodes[i+1]: nodes[0]
        const addedNodes = [startNode, ...cellIntersectedNodesOfEdge2D(startNode, endNode, cellSize)]
        const addedEdges = addedNodes.map((node1, i) => {
            const node2 = i !== addedNodes.length-1 ? addedNodes[i+1]: endNode
            const vec = node2.minus(node1)
            return {
                nodes: [node1, node2],
                norm: !(isCCW && isNormOut)? new Vec2(-vec.y, vec.x).normalize(): new Vec2(vec.y, -vec.x).normalize()
            }
        })
        return {
            nodes: accum.nodes.concat(addedNodes),
            edges: accum.edges.concat(addedEdges)
        }
    }, {nodes: [], edges: []})
    return {
        nodes: network.nodes.map((pos, i) => {return {
            position: pos,
            id: minNodeId + i,
            edges: i !== 0
                ? [network.edges[i - 1], network.edges[i]]
                : [network.edges[network.edges.length - 1], network.edges[0]]
        }}),
        edges: network.edges.map((edge, i) => {return {
            ...edge, 
            id: minEdgeId + i
        }})
    }
}

function polygonsToRegionedNodes (polygons, cellSize) {
    const network = polygons.reduce((network, polygon) => {
        const {nodes, isCCW, isNormOut} = polygon
        const addedNetwork = networkOfPolygonObstacle2D(nodes, isCCW, isNormOut, cellSize, network.nodes.length, network.edges.length)
        return {
            nodes: network.nodes.concat(addedNetwork.nodes),
            edges: network.edges.concat(addedNetwork.edges)
        }
    }, {nodes: [], edges: []})
    return positionedObjectsToRegions(network.nodes, cellSize)
}

function SG_boidWithStaticObs (visualRange, minDistance, maxSpeed, weights, regionedObstacles) {
    return (myState, inputs) => {
        const neighborObstacleNodes = getPossibleNeighbors(myState, regionedObstacles, visualRange)
            .filter(node => Vec.distance(myState.position, node.position) < visualRange)
        const avoidObstaclesForce = visibleEdges(myState.position, neighborObstacleNodes)
            .reduce((sum, edge) => {
                return sum.add(edge.norm)
            }, myState.accelaration.genZero())
        const accelarations = PG_boidForce(visualRange, minDistance, 0, 0, 0, {...weights, keepWithinBound: 0})(myState, inputs)
            .map(tf => [tf[0], tf[1].add(avoidObstaclesForce.scalarMul(weights.avoidObstacles))])
        return {...SG_accelaratablePoint(maxSpeed)(myState, accelarations), id: myState.id}
    }
}

function SG_boidsWithStaticObs (visualRange, minDistance, maxSpeed, weights, polygons) {
    const regionedObstacles = polygonsToRegionedNodes(polygons, visualRange)
    const boidSystem = SG_boidWithStaticObs(visualRange, minDistance, maxSpeed, weights, regionedObstacles)
    return SG_boidsAndRegions(Math.max(visualRange, minDistance), boidSystem)
}