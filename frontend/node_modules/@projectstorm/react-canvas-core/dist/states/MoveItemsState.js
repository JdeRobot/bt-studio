import { AbstractDisplacementState } from '../core-state/AbstractDisplacementState';
import { Action, InputType } from '../core-actions/Action';
import { BasePositionModel } from '../core-models/BasePositionModel';
export class MoveItemsState extends AbstractDisplacementState {
    constructor() {
        super({
            name: 'move-items'
        });
        this.registerAction(new Action({
            type: InputType.MOUSE_DOWN,
            fire: (event) => {
                const element = this.engine.getActionEventBus().getModelForEvent(event);
                if (!element) {
                    return;
                }
                if (!element.isSelected()) {
                    this.engine.getModel().clearSelection();
                }
                element.setSelected(true);
                this.engine.repaintCanvas();
            }
        }));
    }
    activated(previous) {
        super.activated(previous);
        this.initialPositions = {};
    }
    fireMouseMoved(event) {
        const items = this.engine.getModel().getSelectedEntities();
        const model = this.engine.getModel();
        for (let item of items) {
            if (item instanceof BasePositionModel) {
                if (item.isLocked()) {
                    continue;
                }
                if (!this.initialPositions[item.getID()]) {
                    this.initialPositions[item.getID()] = {
                        point: item.getPosition(),
                        item: item
                    };
                }
                const pos = this.initialPositions[item.getID()].point;
                item.setPosition(model.getGridPosition(pos.x + event.virtualDisplacementX), model.getGridPosition(pos.y + event.virtualDisplacementY));
            }
        }
        this.engine.repaintCanvas();
    }
}
//# sourceMappingURL=MoveItemsState.js.map