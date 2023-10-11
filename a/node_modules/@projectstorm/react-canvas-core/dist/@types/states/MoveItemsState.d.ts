import { AbstractDisplacementState, AbstractDisplacementStateEvent } from '../core-state/AbstractDisplacementState';
import { State } from '../core-state/State';
import { BasePositionModel } from '../core-models/BasePositionModel';
import { Point } from '@projectstorm/geometry';
import { CanvasEngine } from '../CanvasEngine';
export declare class MoveItemsState<E extends CanvasEngine = CanvasEngine> extends AbstractDisplacementState<E> {
    initialPositions: {
        [id: string]: {
            point: Point;
            item: BasePositionModel;
        };
    };
    constructor();
    activated(previous: State): void;
    fireMouseMoved(event: AbstractDisplacementStateEvent): void;
}
