import { BaseModel } from './BaseModel';
import { Point, Rectangle } from '@projectstorm/geometry';
export class BasePositionModel extends BaseModel {
    constructor(options) {
        super(options);
        this.position = options.position || new Point(0, 0);
    }
    setPosition(x, y) {
        if (x instanceof Point) {
            this.position = x;
        }
        else {
            this.position = new Point(x, y);
        }
        this.fireEvent({}, 'positionChanged');
    }
    getBoundingBox() {
        return Rectangle.fromPointAndSize(this.position, 0, 0);
    }
    deserialize(event) {
        super.deserialize(event);
        this.position = new Point(event.data.x, event.data.y);
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { x: this.position.x, y: this.position.y });
    }
    getPosition() {
        return this.position;
    }
    getX() {
        return this.position.x;
    }
    getY() {
        return this.position.y;
    }
}
//# sourceMappingURL=BasePositionModel.js.map