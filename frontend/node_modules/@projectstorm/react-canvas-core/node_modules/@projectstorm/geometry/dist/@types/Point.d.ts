import { Matrix } from './Matrix';
export declare class Point {
    x: number;
    y: number;
    constructor(x?: number, y?: number);
    translate(x: number, y: number): void;
    clone(): Point;
    toSVG(): string;
    asMatrix(): Matrix;
    transform(matrix: Matrix): void;
    static middlePoint(pointA: Point, pointB: Point): Point;
}
