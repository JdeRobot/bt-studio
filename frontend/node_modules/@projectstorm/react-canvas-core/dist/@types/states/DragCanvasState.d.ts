import { AbstractDisplacementState, AbstractDisplacementStateEvent } from '../core-state/AbstractDisplacementState';
import { State } from '../core-state/State';
export interface DragCanvasStateOptions {
    /**
     * If enabled, the canvas is available to drag
     */
    allowDrag?: boolean;
}
export declare class DragCanvasState extends AbstractDisplacementState {
    initialCanvasX: number;
    initialCanvasY: number;
    config: DragCanvasStateOptions;
    constructor(options?: DragCanvasStateOptions);
    activated(prev: any): Promise<void>;
    deactivated(next: State): void;
    fireMouseMoved(event: AbstractDisplacementStateEvent): void;
}
