import { Point } from './Point';
import _forEach from 'lodash/forEach';
import _map from 'lodash/map';
import { Matrix } from './Matrix';
import { boundingBoxFromPoints } from './toolkit';
import { BoundsCorner } from './Bounds';
export class Polygon {
    constructor(points = []) {
        this.points = points;
    }
    serialize() {
        return _map(this.points, (point) => {
            return [point.x, point.y];
        });
    }
    deserialize(data) {
        this.points = _map(data, (point) => {
            return new Point(point[0], point[1]);
        });
    }
    scale(x, y, origin) {
        let matrix = Matrix.createScaleMatrix(x, y, origin);
        _forEach(this.points, (point) => {
            point.transform(matrix);
        });
    }
    transform(matrix) {
        _forEach(this.points, (point) => {
            point.transform(matrix);
        });
    }
    setPoints(points) {
        this.points = points;
    }
    getPoints() {
        return this.points;
    }
    rotate(degrees) {
        this.transform(Matrix.createRotateMatrix(degrees / (180 / Math.PI), this.getOrigin()));
    }
    translate(offsetX, offsetY) {
        _forEach(this.points, (point) => {
            point.translate(offsetX, offsetY);
        });
    }
    doClone(ob) {
        this.points = _map(ob.points, (point) => {
            return point.clone();
        });
    }
    clone() {
        let ob = Object.create(this);
        ob.doClone(this);
        return ob;
    }
    getOrigin() {
        if (this.points.length === 0) {
            return null;
        }
        let dimensions = boundingBoxFromPoints(this.points);
        return Point.middlePoint(dimensions[BoundsCorner.TOP_LEFT], dimensions[BoundsCorner.BOTTOM_RIGHT]);
    }
    getBoundingBox() {
        return boundingBoxFromPoints(this.points);
    }
}
//# sourceMappingURL=Polygon.js.map