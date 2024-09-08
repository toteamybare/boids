import { Vec, Vec2} from "./vector.js"

// Size of canvas. These get updated to fill the whole browser.
let width = 1000;
let height = 1000;
let isRunning = false;
let idx = 0;

var boids = [{
    t: 0,
    state: [{
        x: 100,
        y: 100,
        vx: 0,
        vy: 0
    }]
}, {
    t: 1,
    state: [{
        x: 150,
        y: 150,
        vx: 0,
        vy: 0
    }]
}]

const range = (n, start=0) => Array.from({length: n}, (_, i) => start + i)

const circle1 = {
    nodes: range(12).map(n => {
        return new Vec2(Math.cos(2 * Math.PI / 12 * n) * 400 + 500, Math.sin(2 * Math.PI / 12 * n) * 400 + 500)
    }),
    isCCW: true,
    isNormOut: false
}
const circle2 = {
    nodes: range(12).map(n => {
        return new Vec2(Math.cos(2 * Math.PI / 12 * n) * 430 + 500, Math.sin(2 * Math.PI / 12 * n) * 430 + 500)
    }),
    isCCW: true,
    isNormOut: false
}

let square = {
    nodes: [new Vec2(0, 0), new Vec2(1000, 0), new Vec2(1000, 1000), new Vec2(0, 1000)],
    isCCW: true,
    isNormOut: false
}

// Called initially and whenever the window resizes to update the canvas
// size and width/height variables.
function sizeCanvas() {
    const canvas = document.getElementById("boids");
    // width = window.innerWidth;
    // height = window.innerHeight;
    canvas.width = width;
    canvas.height = height;
    console.log(width)
    console.log(height)
}

function loadFile(e) {
    console.log("loading file");
    const reader = new FileReader();
    reader.addEventListener("load", (e) => {
        let result = JSON.parse(e.target.result);
        boids = result;
        idx=0;
        drawAll();
    })
    reader.readAsText(e.target.files[0]);
}

const DRAW_TRAIL = false;

function drawBoid(ctx, boid) {
    const angle = Math.atan2(boid.vy, boid.vx);
    ctx.translate(boid.x, boid.y);
    ctx.rotate(angle);
    ctx.translate(-boid.x, -boid.y);
    ctx.fillStyle = "#558cf4";
    ctx.beginPath();
    ctx.moveTo(boid.x, boid.y);
    ctx.lineTo(boid.x - 15, boid.y + 5);
    ctx.lineTo(boid.x - 15, boid.y - 5);
    ctx.lineTo(boid.x, boid.y);
    ctx.fill();
    ctx.setTransform(1, 0, 0, 1, 0, 0);

    if (DRAW_TRAIL) {
        ctx.strokeStyle = "#558cf466";
        ctx.beginPath();
        ctx.moveTo(boid.history[0][0], boid.history[0][1]);
        for (const point of boid.history) {
            ctx.lineTo(point[0], point[1]);
        }
        ctx.stroke();
    }
}

function drawPolygon(ctx, nodes) {
    ctx.lineWidth = 2;
    ctx.strokeStyle = "orange";
    ctx.beginPath()
    ctx.moveTo(nodes[0].x, nodes[0].y)
    nodes.slice(0).forEach((node) => {
        ctx.lineTo(node.x, node.y)
    })
    ctx.lineTo(nodes[0].x, nodes[0].y)
    ctx.stroke()
}

function drawAll() {
    // Clear the canvas and redraw all the boids in their current positions
    const ctx = document.getElementById("boids").getContext("2d");
    ctx.clearRect(0, 0, width, height);
    for (let boid of boids[idx].state) {
        drawBoid(ctx, boid);
    }
    // for (let obstacle of [
    //     circle1,
    //     circle2
    // ]) {
    //     drawPolygon(ctx, obstacle.nodes)
    // }

    document.getElementById("timeTxt").innerText = boids[idx].t
}

const fps = 30;
let then = Date.now();

// Main animation loop
function animationLoop() {
    let now = Date.now();
    let elapsed = now - then

    if (isRunning && elapsed > 1000/fps) {
        then = now

        drawAll();
    
        idx++;
        if (idx >= boids.length) {
            idx = 0;
        }
    
    }
    // Schedule the next frame
    window.requestAnimationFrame(animationLoop);
}

window.onload = () => {
    // Make sure the canvas always fills the whole window
    // window.addEventListener("resize", sizeCanvas, false);
    sizeCanvas();

    // Schedule the main animation loop
    window.requestAnimationFrame(animationLoop);
    document.getElementById("startBtn").addEventListener("click", () => {isRunning = !isRunning});  
    document.getElementById("simDataInput").addEventListener("change", (e) => {loadFile(e)})
    document.getElementById("resetBtn").addEventListener("click", () => {idx=0; drawAll();});
};
