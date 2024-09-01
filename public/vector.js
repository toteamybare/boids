
export class Vec {
    scalarMul(c) {
        return this.map(val => c*val)
    }
    sqrNorm() {
        return this.dot(this)
    }
    norm() {
        return Math.sqrt(this.dot(this))
    }
    normalize() {
        return this.norm() !== 0 ? this.scalarMul(1/ this.norm()): undefined
    }
    static add (a, b) {
        return a.add(b)
    }
    static minus(a, b) {
        return a.minus(b)
    }
    static dot(a, b) {
        return a.dot(b)
    }
    static distance(a, b) {
        return a.minus(b).norm()
    }
    static sum(arr) {
        return arr.length? arr.reduce((accum, current) => accum.add(current), arr[0].genZero()): undefined
    }
    static ave(arr) {
        return arr.length? Vec.sum(arr).scalarMul(1/arr.length) : undefined
    }
}

export class Vec1 extends Vec {
    constructor(x){
        super()
        this.x = x
    }
    equals(other) {
        return this.x === other.x
    }
    add(other) {
        return new Vec1(this.x + other.x)
    }
    minus(other) {
        return new Vec1(this.x - other.x)
    }
    dot(other) {
        return this.x * other.x
    }
    clone() {
        return new Vec1(this.x)
    }
    genZero() {
        return Vec1.zero()
    }
    map(func) {
        return new Vec1(func (this.x))
    }
    static zero() {
        return new Vec1(0)
    }
}

export class Vec2 extends Vec {
    constructor(x, y){
        super()
        this.x = x
        this.y = y
    }
    equals(other) {
        return this.x === other.x && this.y === other.y
    }
    add(other) {
        return new Vec2(this.x + other.x, this.y + other.y)
    }
    minus(other) {
        return new Vec2(this.x - other.x, this.y - other.y)
    }
    dot(other) {
        return this.x * other.x + this.y * other.y
    }
    clone() {
        return new Vec2(this.x, this.y)
    }
    genZero() {
        return new Vec2(0, 0)
    }
    map(func) {
        return new Vec2(func(this.x), func(this.y))
    }
    static zero() {
        return new Vec2(0, 0)
    }
}

export class Vec3 extends Vec {
    constructor(x, y, z){
        super()
        this.x = x
        this.y = y
        this.z = z
    }
    equals(other) {
        return this.x === other.x && this.y === other.y && this.z === other.z
    }
    add(other) {
        return new Vec3(this.x + other.x, this.y + other.y, this.z + other.z)
    }
    minus(other) {
        return new Vec3(this.x - other.x, this.y - other.y, this.z - other.z)
    }
    dot(other) {
        return this.x * other.x + this.y * other.y + this.z * other.z
    }
    clone() {
        return new Vec3(this.x, this.y, this.z)
    }
    genZero() {
        return Vec3.zero()
    }
    map(func) {
        return new Vec3(func(this.x), func(this.y), func(this.z))
    }
    static zero() {
        return new Vec3(0, 0, 0)
    }
}