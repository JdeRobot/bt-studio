import { Point } from './Point';
import { Polygon } from './Polygon';
import { Bounds } from './Bounds';
export declare const boundingBoxFromPoints: (points: Point[]) => Bounds;
export declare const boundingBoxFromPolygons: (polygons: Polygon[]) => Bounds;
