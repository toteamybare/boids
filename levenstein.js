module.exports = function (a, b) {
    const [an, bn] = [a.length, b.length]
    const initial = [...Array(bn + 1).keys()]

    let dpMemo = initial
    let dpCurrent = []

    for (let ai = 1; ai <= an; ai += 1) {
        dpCurrent.push(ai)
        for (let bi = 1; bi <= bn; bi += 1){
            const same = a[ai-1] == b[bi-1]? 0: 1
            dpCurrent.push(
                Math.min(
                    dpMemo[bi-1] + same,
                    dpMemo[bi] + 1,
                    dpCurrent.at(-1) + 1
                )
            )
        }
        dpMemo = dpCurrent
        dpCurrent = []
    }

    return dpMemo.at(-1)
}