import { DefaultLinkModel } from '@projectstorm/react-diagrams-defaults';
import { RightAngleLinkFactory } from './RightAngleLinkFactory';
export class RightAngleLinkModel extends DefaultLinkModel {
    constructor(options = {}) {
        super(Object.assign({ type: RightAngleLinkFactory.NAME }, options));
        this.lastHoverIndexOfPath = 0;
        this._lastPathXdirection = false;
        this._firstPathXdirection = false;
    }
    setFirstAndLastPathsDirection() {
        let points = this.getPoints();
        for (let i = 1; i < points.length; i += points.length - 2) {
            let dx = Math.abs(points[i].getX() - points[i - 1].getX());
            let dy = Math.abs(points[i].getY() - points[i - 1].getY());
            if (i - 1 === 0) {
                this._firstPathXdirection = dx > dy;
            }
            else {
                this._lastPathXdirection = dx > dy;
            }
        }
    }
    // @ts-ignore
    addPoint(pointModel, index = 1) {
        // @ts-ignore
        super.addPoint(pointModel, index);
        this.setFirstAndLastPathsDirection();
        return pointModel;
    }
    deserialize(event) {
        super.deserialize(event);
        this.setFirstAndLastPathsDirection();
    }
    setManuallyFirstAndLastPathsDirection(first, last) {
        this._firstPathXdirection = first;
        this._lastPathXdirection = last;
    }
    getLastPathXdirection() {
        return this._lastPathXdirection;
    }
    getFirstPathXdirection() {
        return this._firstPathXdirection;
    }
    setWidth(width) {
        this.options.width = width;
        this.fireEvent({ width }, 'widthChanged');
    }
    setColor(color) {
        this.options.color = color;
        this.fireEvent({ color }, 'colorChanged');
    }
}
//# sourceMappingURL=RightAngleLinkModel.js.map