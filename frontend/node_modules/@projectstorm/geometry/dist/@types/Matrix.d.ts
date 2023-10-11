import { Point } from './Point';
export declare class Matrix {
    matrix: number[][];
    constructor(matrix: number[][]);
    mmul(matrix: Matrix): Matrix;
    asArray(): number[][];
    get(rowIndex: number, columnIndex: number): number;
    static multiply(...matrices: Matrix[]): Matrix;
    static scaleMatrix(x: number, y: number): Matrix;
    static translateMatrix(x: number, y: number): Matrix;
    static rotateMatrix(deg: number): Matrix;
    static createScaleMatrix(x: any, y: any, origin: Point): Matrix;
    static createRotateMatrix(deg: number, origin: Point): Matrix;
}
