import { Point } from './Point';
import { Polygon } from './Polygon';
import { BoundsCorner, boundsFromPositionAndSize, createEmptyBounds } from './Bounds';
export class Rectangle extends Polygon {
    static fromPositionAndSize(x, y, width, height) {
        return new Rectangle(boundsFromPositionAndSize(x, y, width, height));
    }
    static fromPointAndSize(position, width, height) {
        return new Rectangle(boundsFromPositionAndSize(position.x, position.y, width, height));
    }
    constructor(points) {
        if (!points) {
            points = createEmptyBounds();
        }
        super([
            points[BoundsCorner.TOP_LEFT],
            points[BoundsCorner.TOP_RIGHT],
            points[BoundsCorner.BOTTOM_RIGHT],
            points[BoundsCorner.BOTTOM_LEFT]
        ]);
    }
    updateDimensions(x, y, width, height) {
        const points = boundsFromPositionAndSize(x, y, width, height);
        this.setPoints([
            points[BoundsCorner.TOP_LEFT],
            points[BoundsCorner.TOP_RIGHT],
            points[BoundsCorner.BOTTOM_RIGHT],
            points[BoundsCorner.BOTTOM_LEFT]
        ]);
    }
    setPoints(points) {
        if (points.length !== 4) {
            throw 'Rectangles must always have 4 points';
        }
        super.setPoints(points);
    }
    containsPoint(point) {
        const tl = this.getTopLeft();
        const br = this.getBottomRight();
        return point.x >= tl.x && point.x <= br.x && point.y >= tl.y && point.y <= br.y;
    }
    getWidth() {
        return Math.sqrt(Math.pow(this.getTopLeft().x - this.getTopRight().x, 2) + Math.pow(this.getTopLeft().y - this.getTopRight().y, 2));
    }
    getHeight() {
        return Math.sqrt(Math.pow(this.getBottomLeft().x - this.getTopLeft().x, 2) +
            Math.pow(this.getBottomLeft().y - this.getTopLeft().y, 2));
    }
    getTopMiddle() {
        return Point.middlePoint(this.getTopLeft(), this.getTopRight());
    }
    getBottomMiddle() {
        return Point.middlePoint(this.getBottomLeft(), this.getBottomRight());
    }
    getLeftMiddle() {
        return Point.middlePoint(this.getBottomLeft(), this.getTopLeft());
    }
    getRightMiddle() {
        return Point.middlePoint(this.getBottomRight(), this.getTopRight());
    }
    getTopLeft() {
        return this.points[0];
    }
    getTopRight() {
        return this.points[1];
    }
    getBottomRight() {
        return this.points[2];
    }
    getBottomLeft() {
        return this.points[3];
    }
}
//# sourceMappingURL=Rectangle.js.map