import { Point } from './Point';
import { Polygon } from './Polygon';
import { Bounds } from './Bounds';
export declare class Rectangle extends Polygon {
    static fromPositionAndSize(x: number, y: number, width: number, height: number): Rectangle;
    static fromPointAndSize(position: Point, width: number, height: number): Rectangle;
    constructor(points?: Bounds);
    updateDimensions(x: number, y: number, width: number, height: number): void;
    setPoints(points: Point[]): void;
    containsPoint(point: Point): boolean;
    getWidth(): number;
    getHeight(): number;
    getTopMiddle(): Point;
    getBottomMiddle(): Point;
    getLeftMiddle(): Point;
    getRightMiddle(): Point;
    getTopLeft(): Point;
    getTopRight(): Point;
    getBottomRight(): Point;
    getBottomLeft(): Point;
}
