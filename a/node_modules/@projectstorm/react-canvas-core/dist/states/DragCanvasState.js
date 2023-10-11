var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
import { AbstractDisplacementState } from '../core-state/AbstractDisplacementState';
export class DragCanvasState extends AbstractDisplacementState {
    constructor(options = {}) {
        super({
            name: 'drag-canvas'
        });
        this.config = Object.assign({ allowDrag: true }, options);
    }
    activated(prev) {
        const _super = Object.create(null, {
            activated: { get: () => super.activated }
        });
        return __awaiter(this, void 0, void 0, function* () {
            _super.activated.call(this, prev);
            this.engine.getModel().clearSelection();
            yield this.engine.repaintCanvas(true);
            // we can block layer rendering because we are only targeting the transforms
            for (let layer of this.engine.getModel().getLayers()) {
                layer.allowRepaint(false);
            }
            this.initialCanvasX = this.engine.getModel().getOffsetX();
            this.initialCanvasY = this.engine.getModel().getOffsetY();
        });
    }
    deactivated(next) {
        super.deactivated(next);
        for (let layer of this.engine.getModel().getLayers()) {
            layer.allowRepaint(true);
        }
    }
    fireMouseMoved(event) {
        if (this.config.allowDrag) {
            this.engine
                .getModel()
                .setOffset(this.initialCanvasX + event.displacementX, this.initialCanvasY + event.displacementY);
            this.engine.repaintCanvas();
        }
    }
}
//# sourceMappingURL=DragCanvasState.js.map