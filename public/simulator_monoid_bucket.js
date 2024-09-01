import {Vec, Vec2} from "./vector.js"

function S_timeSystem(current, elapsed) {
    return current + elapsed.num
}

function SG_accelaratablePoint(maxSpeed = undefined) {
    return  (spacialProperties, inputs) => {
        let newPos = spacialProperties.position.clone()
        let newVel = spacialProperties.velocity.clone()
        let sumTime = 0
        inputs.list.forEach((input) => {
            let [time, acc] = input
            sumTime += time
            newVel = newVel.add(acc.scalarMul(time))
            const speed = newVel.norm()
            newVel = (maxSpeed != undefined && speed > maxSpeed)? newVel.scalarMul(maxSpeed/speed): newVel
            newPos = newPos.add(newVel.scalarMul(time))
        })
        const newAcc = sumTime? newVel.minus(spacialProperties.velocity).scalarMul(1/sumTime): spacialProperties.accelaration.genZero()
        return {position: newPos, velocity: newVel, accelaration: newAcc}
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
            {t: 0, boids: initialBoids, regions: positionedObjectsToRegions(initialBoids, cellSize)},
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
        let {x, y} = hash(obj.position)
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