import { AbstractDisplacementState } from '../core-state/AbstractDisplacementState';
import { SelectionLayerModel } from '../entities/selection/SelectionLayerModel';
import { Rectangle } from '@projectstorm/geometry';
export class SelectionBoxState extends AbstractDisplacementState {
    constructor() {
        super({
            name: 'selection-box'
        });
    }
    activated(previous) {
        super.activated(previous);
        this.layer = new SelectionLayerModel();
        this.engine.getModel().addLayer(this.layer);
    }
    deactivated(next) {
        super.deactivated(next);
        this.layer.remove();
        this.engine.repaintCanvas();
    }
    getBoxDimensions(event) {
        let rel;
        if ('touches' in event.event) {
            const touch = event.event.touches[0];
            rel = this.engine.getRelativePoint(touch.clientX, touch.clientY);
        }
        else {
            rel = this.engine.getRelativePoint(event.event.clientX, event.event.clientY);
        }
        return {
            left: rel.x > this.initialXRelative ? this.initialXRelative : rel.x,
            top: rel.y > this.initialYRelative ? this.initialYRelative : rel.y,
            width: Math.abs(rel.x - this.initialXRelative),
            height: Math.abs(rel.y - this.initialYRelative),
            right: rel.x < this.initialXRelative ? this.initialXRelative : rel.x,
            bottom: rel.y < this.initialYRelative ? this.initialYRelative : rel.y
        };
    }
    fireMouseMoved(event) {
        this.layer.setBox(this.getBoxDimensions(event));
        const relative = this.engine.getRelativeMousePoint({
            clientX: this.initialX,
            clientY: this.initialY
        });
        if (event.virtualDisplacementX < 0) {
            relative.x -= Math.abs(event.virtualDisplacementX);
        }
        if (event.virtualDisplacementY < 0) {
            relative.y -= Math.abs(event.virtualDisplacementY);
        }
        const rect = Rectangle.fromPointAndSize(relative, Math.abs(event.virtualDisplacementX), Math.abs(event.virtualDisplacementY));
        for (let model of this.engine.getModel().getSelectionEntities()) {
            if (model.getBoundingBox) {
                const bounds = model.getBoundingBox();
                if (rect.containsPoint(bounds.getTopLeft()) && rect.containsPoint(bounds.getBottomRight())) {
                    model.setSelected(true);
                }
                else {
                    model.setSelected(false);
                }
            }
        }
        this.engine.repaintCanvas();
    }
}
//# sourceMappingURL=SelectionBoxState.js.map