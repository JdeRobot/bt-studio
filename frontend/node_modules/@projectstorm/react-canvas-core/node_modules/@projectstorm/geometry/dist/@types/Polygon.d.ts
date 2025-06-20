import { Point } from './Point';
import { Matrix } from './Matrix';
import { Bounds } from './Bounds';
export declare class Polygon {
    protected points: Point[];
    constructor(points?: Point[]);
    serialize(): number[][];
    deserialize(data: any): void;
    scale(x: any, y: any, origin: Point): void;
    transform(matrix: Matrix): void;
    setPoints(points: Point[]): void;
    getPoints(): Point[];
    rotate(degrees: number): void;
    translate(offsetX: number, offsetY: number): void;
    doClone(ob: this): void;
    clone(): this;
    getOrigin(): Point;
    getBoundingBox(): Bounds;
}
