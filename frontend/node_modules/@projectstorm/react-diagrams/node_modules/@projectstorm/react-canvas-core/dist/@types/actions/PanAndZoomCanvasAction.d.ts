import { Action } from '../core-actions/Action';
export interface PanAndZoomCanvasActionOptions {
    inverseZoom?: boolean;
}
export declare class PanAndZoomCanvasAction extends Action {
    constructor(options?: PanAndZoomCanvasActionOptions);
}
