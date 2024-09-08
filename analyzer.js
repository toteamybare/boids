const espree = require("espree");
const levenstein = require ("./levenstein.js")
const fs = require('fs');

// see https://github.com/eslint/js/blob/main/packages/espree/lib/token-translator.js
const tokenTypes = {Boolean: "Boolean",
    EOF: "<end>",
    Identifier: "Identifier",
    PrivateIdentifier: "PrivateIdentifier",
    Keyword: "Keyword",
    Null: "Null",
    Numeric: "Numeric",
    Punctuator: "Punctuator",
    String: "String",
    RegularExpression: "RegularExpression",
    Template: "Template",
    JSXIdentifier: "JSXIdentifier",
    JSXText: "JSXText"
}

// see https://github.com/estree/estree/blob/master/es5.md
const nodeTypes = {
    Declaration: ["FunctionDeclaration", "VariableDeclaration", "ClassDeclaration"],
    Exports: ["ExportNamedDeclaration", "ExportDefaultDeclaration"],
    Loops: ["WhileStatement", "DoWhileStatement", "ForStatement", "ForInStatement", "ForOfStatement"],
    Choice: ["IfStatement", "SwitchStatement"]
}

function typeFrequencyOfTokens(tokens) {
    let result = Object.keys(tokenTypes).reduce((accum, key) => {
        accum[tokenTypes[key]] = 0
        return accum
    }, {})

    return tokens.reduce((accum, token) => {
        accum[token.type] += 1
        return accum
    }, result)
}

function countSpecificToken(tokens, specificToken) {
    return tokens.reduce((accum, token) => {
        const add = token.value === specificToken? 1: 0
        return accum + add
    }, 0)
}

function createEstree(codeStr) {
    const stats = {code: codeStr}
    try {
        stats.ast = espree.parse(codeStr, { ecmaVersion: "latest", sourceType: "module"});
    } catch (error) {
        console.log(error)
        stats.ast = null
    }
    stats.tokens = espree.tokenize(codeStr, { ecmaVersion: "latest", sourceType: "module"});
    return stats
}

function codeStats (codeStr, analyzeFuncs = true) {
    const physicalLines = codeStr.split("\n");
    const logicalLines = physicalLines.filter((line) => ! (/^(\s*)((\/\/)|$)/.test(line)))

    const estree = createEstree(codeStr)
    const tokenFrequency = typeFrequencyOfTokens(estree.tokens)
    const numTokens = Object.values(tokenFrequency).reduce((sum, num) => sum+num, 0)
    const numLTokens = Object.entries(tokenFrequency).reduce((sum, entry) => {
        const [key, value] = entry
        const add = !(["<end>", "Punctuator"].includes(key)) ? value: 0
        return sum + add
    }, 0)
    const CC = countSpecificToken(estree.tokens, "while")
        + countSpecificToken(estree.tokens, "for")
        + countSpecificToken(estree.tokens, "if")
        + countSpecificToken(estree.tokens, "case")
        + countSpecificToken(estree.tokens, "?")
        + 1


    let topLevelDecs = undefined
    if (analyzeFuncs) {
        topLevelDecs = estree.ast.body.filter(node =>  node.type != "ImportDeclaration").reduce((accum, node) => {
            if (nodeTypes.Exports.includes(node.type)) accum.push(node.declaration)
            else accum.push(node)
            return accum
        }, []).reduce((accum, node) => {
            if (node.type == "VariableDeclaration") accum = accum.concat(node.declarations)
            else accum.push(node)
            return accum
        }, []).map( (node) =>  {
            return {  
                name: node.id.name,
                called: countSpecificToken(estree.tokens, node.id.name)-1,
                stats: codeStats(codeStr.slice(node.start, node.end), false)
            }
        })
    }

    return {
        LOC: physicalLines.length,
        LLOC: logicalLines.length,
        numTokens: numTokens,
        numLTokens: numLTokens,
        CC: CC,
        perD: {
            LOC: analyzeFuncs? round(avgObjs(topLevelDecs, dec => dec.stats.LOC), 1): undefined,
            LLOC: analyzeFuncs? round(avgObjs(topLevelDecs, dec => dec.stats.LLOC), 1): undefined,
            numTokens: analyzeFuncs? round(avgObjs(topLevelDecs, dec => dec.stats.numTokens), 1): undefined,
            numLTokens: analyzeFuncs? round(avgObjs(topLevelDecs, dec => dec.stats.numLTokens), 1): undefined,
            CC: analyzeFuncs? round(avgObjs(topLevelDecs, dec => dec.stats.CC), 1): undefined,
            called: analyzeFuncs? round(avgObjs(topLevelDecs, dec => dec.called), 1): undefined,
        },
        numTopLevelDecs: analyzeFuncs? topLevelDecs.length: undefined,
        topLevelDecs: topLevelDecs,
        tokenFrequency: tokenFrequency,
        estree: estree
    }
}

// assume that top-level declarations in stats2 is monotonically increased from that in stats1
function codeDifference(stats1, stats2) {
    const levDistance = levenstein(mapWithKey(stats1.estree.tokens, "value"), mapWithKey(stats2.estree.tokens, "value"))
    const topLevelDecs = stats2.topLevelDecs.map((dec2, i) => {
        const dec1 = stats1.topLevelDecs[i]
        const leven = dec1 !== undefined? 
            levenstein( mapWithKey(dec2.stats.estree.tokens , "value"), mapWithKey(dec1.stats.estree.tokens, "value"))
            : dec2.stats.numTokens
        return {
            name: dec2.name,
            category: (dec1 === undefined)? "new": (leven === 0)? "unmodified": "modified",
            leven: leven,
            dcalled: dec2.called - (dec1 === undefined? 0:dec1.called)
        }
    })

    return {
        levDistance: levDistance,
        numNewDecs: topLevelDecs.reduce((sum, dec) => sum + (dec.category === "new"? 1: 0), 0),
        numModDecs: topLevelDecs.reduce((sum, dec) => sum + (dec.category === "modified"? 1: 0), 0),
        numUnmodDecs: topLevelDecs.reduce((sum, dec) => sum + (dec.category === "unmodified"? 1: 0), 0),
        numReuses: topLevelDecs.reduce((sum, dec) => sum + (dec.category !== "new"? dec.dcalled: 0), 0),
        topLevelDecs: topLevelDecs
    }
}

function mapWithKey(array, key) {
    return array.map(obj => obj[key])
}

function avgObjs (objs, quantify) {
    const sum = objs.reduce((accum, obj) => accum + quantify(obj), 0)
    return objs.length? sum/objs.length:0
}

function round (number, decimal) {
    const factor = Math.pow(10, decimal)
    return Math.round(number * factor) / factor
}

function showAnalysis(analysis) {
    const shown = {...analysis}
    delete shown.estree
    delete shown.topLevelDecs
    delete shown.tokenFrequency
    console.log(shown)
}

// console.log(typeFrequencyOfTokens(control.tokens))
// console.log(typeFrequencyOfTokens(monoid.tokens))

const control = codeStats(fs.readFileSync("./public/simulator_control.js", "utf-8"))
const controlB = codeStats(fs.readFileSync("./public/simulator_control_bucket.js", "utf-8"))
const controlO = codeStats(fs.readFileSync("./public/simulator_control_obstacles.js", "utf-8"))
const monoid = codeStats(fs.readFileSync("./public/simulator_monoid.js", "utf-8"))
const monoidB = codeStats(fs.readFileSync("./public/simulator_monoid_bucket.js", "utf-8"))
const monoidO = codeStats(fs.readFileSync("./public/simulator_monoid_obstacles.js", "utf-8"))
// const control2 = createEstree(fs.readFileSync("./public/simulator_control2.js", "utf-8"))
// const test1 = createEstree('function pn (x) {if (x>0) {return "positive"} else if (x<0) {return "negative"} else {return "zero"}}')
// const test2 = createEstree('function pn (x) {switch (x) {case 1: return "one"; case 2: return "two"}}')
// const test2 = createEstree('function hoge () { console.log(hoge) }')
// console.log(control.ast.body[1].id)

// console.log(JSON.stringify(codeStats(control.code, true)))
// console.log(JSON.stringify(control.ast.body))
// console.log(JSON.stringify(control.tokens))
// console.log(control.ast)
// showAnalysis(monoid)
// showAnalysis(control)
// showAnalysis(monoidB)
// showAnalysis(controlB)
// showAnalysis(monoidO)
// showAnalysis(controlO)
console.log(codeDifference(monoid, monoidB))
console.log(codeDifference(control, controlB))
console.log(codeDifference(monoidB, monoidO))
console.log(codeDifference(controlB, controlO))
// console.log(JSON.stringify(test2.ast.body))