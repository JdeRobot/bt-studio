import { Point } from './Point';
import { Polygon } from './Polygon';
export var BezierCurvepPoints;
(function (BezierCurvepPoints) {
    BezierCurvepPoints[BezierCurvepPoints["SOURCE"] = 0] = "SOURCE";
    BezierCurvepPoints[BezierCurvepPoints["SOURCE_CONTROL"] = 1] = "SOURCE_CONTROL";
    BezierCurvepPoints[BezierCurvepPoints["TARGET_CONTROL"] = 2] = "TARGET_CONTROL";
    BezierCurvepPoints[BezierCurvepPoints["TARGET"] = 3] = "TARGET";
})(BezierCurvepPoints || (BezierCurvepPoints = {}));
export class BezierCurve extends Polygon {
    constructor() {
        super([new Point(0, 0), new Point(0, 0), new Point(0, 0), new Point(0, 0)]);
    }
    getSVGCurve() {
        return `M${this.getSource().toSVG()} C${this.getSourceControl().toSVG()}, ${this.getTargetControl().toSVG()}, ${this.getTarget().toSVG()}`;
    }
    setPoints(points) {
        if (points.length !== 4) {
            throw new Error('BezierCurve must have extactly 4 points');
        }
        super.setPoints(points);
    }
    getSource() {
        return this.points[BezierCurvepPoints.SOURCE];
    }
    getSourceControl() {
        return this.points[BezierCurvepPoints.SOURCE_CONTROL];
    }
    getTargetControl() {
        return this.points[BezierCurvepPoints.TARGET_CONTROL];
    }
    getTarget() {
        return this.points[BezierCurvepPoints.TARGET];
    }
    setSource(point) {
        this.points[BezierCurvepPoints.SOURCE] = point;
    }
    setSourceControl(point) {
        this.points[BezierCurvepPoints.SOURCE_CONTROL] = point;
    }
    setTargetControl(point) {
        this.points[BezierCurvepPoints.TARGET_CONTROL] = point;
    }
    setTarget(point) {
        this.points[BezierCurvepPoints.TARGET] = point;
    }
}
//# sourceMappingURL=BezierCurve.js.map