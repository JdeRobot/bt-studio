import { AbstractDisplacementState, AbstractDisplacementStateEvent } from '../core-state/AbstractDisplacementState';
import { State } from '../core-state/State';
import { SelectionLayerModel } from '../entities/selection/SelectionLayerModel';
import { CanvasEngine } from '../CanvasEngine';
export interface SimpleClientRect {
    left: number;
    right: number;
    width: number;
    height: number;
    top: number;
    bottom: number;
}
export declare class SelectionBoxState<E extends CanvasEngine = CanvasEngine> extends AbstractDisplacementState<E> {
    layer: SelectionLayerModel;
    constructor();
    activated(previous: State): void;
    deactivated(next: State): void;
    getBoxDimensions(event: AbstractDisplacementStateEvent): SimpleClientRect;
    fireMouseMoved(event: AbstractDisplacementStateEvent): void;
}
