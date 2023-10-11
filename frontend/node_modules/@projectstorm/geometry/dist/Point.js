import { Matrix } from './Matrix';
export class Point {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }
    translate(x, y) {
        this.x += x;
        this.y += y;
    }
    clone() {
        return new Point(this.x, this.y);
    }
    toSVG() {
        return this.x + ' ' + this.y;
    }
    asMatrix() {
        return new Matrix([[this.x], [this.y], [1]]);
    }
    transform(matrix) {
        let final = matrix.mmul(this.asMatrix());
        this.x = final.get(0, 0);
        this.y = final.get(1, 0);
    }
    static middlePoint(pointA, pointB) {
        return new Point((pointB.x + pointA.x) / 2, (pointB.y + pointA.y) / 2);
    }
}
//# sourceMappingURL=Point.js.map