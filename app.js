const express = require("express")
const app = express()

app.use(express.static("./public"))

const server = app.listen(3000, () => {
    console.log("running at 3000")
})