function timeSystem(current, elapsed) {
    return current + elapsed.num
}

function newtonianSystemEuler (xv, accelarations) {
    let [location, velocity] = xv
    accelarations.list.forEach((elem) => {
        let [time, acc] = elem
        velocity += time * acc
        location += time * velocity
    })
    return [location, velocity]
}

function composeSystems(transitions, projections, propagations, compose) {
    return function (state, input) {
        let newStates = transitions.map((transition, i) => {
            return transition(projections[i](state), propagations[i](state, input))
        })
        return compose(newStates)
    }
}

const timeAndObjectSystem = composeSystems(
    [timeSystem, newtonianSystemEuler],
    [(arr) => arr[0], (arr) => arr[1]],
    [(_, accs) => new MNum(accs.list.reduce((accum, cur) => accum + cur[0], 0)), (_, accs) => accs],
    identity
)

function closedSimulation (transition, inputGenerator, toContinue, initialState, sumInput) {
    function sim(transition, inputGenerator, toContinue, results) {
        let [lastState, sumInput] = results.at(-1)
        let newInput = inputGenerator(lastState, sumInput)
        let newState = transition(lastState, newInput)
        let newSumInput = sumInput.add(newInput)
        if (toContinue(newState, newSumInput)) {
            return sim(transition, inputGenerator, toContinue, [...results, [newState, newSumInput]])
        } else  {
            return results;
        }
    }
    return sim(transition, inputGenerator, toContinue, [[initialState, sumInput]])
}

class MNum {
    constructor(num) {
        this.num = num
    }
    add(second) {
        return new MNum(this.num + second.num)
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
    static zero() {
        return new MList([])
    }
}

function identity(value) {
    return value
}

function constant(value){
    return function inputGenerator() {
        return value
    }
}

function timeLimit(tmax) {
    return function toContinue(_, sumInput){
        return sumInput.num <= tmax
    }
}

function timeLimitTuple(tmax) {
    return function toContinue(_, sumInput){
        const accumTime = sumInput.list.reduce((accum, current) => {
            let [time, _] = current
            return accum + time
        }, 0)
        return accumTime <= tmax
    }
}

export function test(){
    const result = closedSimulation(
        timeAndObjectSystem,
        constant(new MList([[1, 0.1]])),
        timeLimitTuple(10),
        [0, [0, 0]],
        MList.zero()
    )
    console.log(result)
}