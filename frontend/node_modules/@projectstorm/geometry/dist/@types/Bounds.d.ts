import { Point } from './Point';
export declare enum BoundsCorner {
    TOP_LEFT = "TL",
    TOP_RIGHT = "TR",
    BOTTOM_RIGHT = "BR",
    BOTTOM_LEFT = "BL"
}
export type Bounds = {
    [k in BoundsCorner]: Point;
};
export declare const boundsFromPositionAndSize: (x: number, y: number, width: number, height: number) => Bounds;
export declare const createEmptyBounds: () => {
    TL: Point;
    TR: Point;
    BR: Point;
    BL: Point;
};
