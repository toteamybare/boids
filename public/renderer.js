// Size of canvas. These get updated to fill the whole browser.
let width = 500;
let height = 500;
let isRunning = false;
let idx = 0;

const numBoids = 100;
const visualRange = 75;

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
}];

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

function drawAll() {
    // Clear the canvas and redraw all the boids in their current positions
    const ctx = document.getElementById("boids").getContext("2d");
    ctx.clearRect(0, 0, width, height);
    for (let boid of boids[idx].state) {
        drawBoid(ctx, boid);
    }

    document.getElementById("timeTxt").innerText = boids[idx].t
}

const fps = 30;
let then = Date.now();

// Main animation loop
function animationLoop() {
    now = Date.now();
    elapsed = now - then

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
