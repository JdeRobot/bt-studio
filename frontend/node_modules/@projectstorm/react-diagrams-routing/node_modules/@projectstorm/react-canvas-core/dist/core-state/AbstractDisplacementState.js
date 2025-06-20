import { State } from './State';
import { Action, InputType } from '../core-actions/Action';
export class AbstractDisplacementState extends State {
    constructor(options) {
        super(options);
        this.registerAction(new Action({
            type: InputType.MOUSE_DOWN,
            fire: (actionEvent) => {
                const { clientX, clientY } = actionEvent.event;
                this.handleMoveStart(clientX, clientY);
            }
        }));
        this.registerAction(new Action({
            type: InputType.MOUSE_MOVE,
            fire: (actionEvent) => {
                const { event } = actionEvent;
                if (event.buttons === 0) {
                    // If buttons is 0, it means the mouse is not down, the user may have released it
                    // outside of the canvas, then we eject the state
                    this.eject();
                    return;
                }
                const { clientX, clientY } = event;
                this.handleMove(clientX, clientY, event);
            }
        }));
        this.registerAction(new Action({
            type: InputType.MOUSE_UP,
            fire: () => this.handleMoveEnd()
        }));
        this.registerAction(new Action({
            type: InputType.TOUCH_START,
            fire: (actionEvent) => {
                const { clientX, clientY } = actionEvent.event.touches[0];
                this.handleMoveStart(clientX, clientY);
            }
        }));
        this.registerAction(new Action({
            type: InputType.TOUCH_MOVE,
            fire: (actionEvent) => {
                const { event } = actionEvent;
                const { clientX, clientY } = event.touches[0];
                this.handleMove(clientX, clientY, event);
            }
        }));
        this.registerAction(new Action({
            type: InputType.TOUCH_END,
            fire: () => this.handleMoveEnd()
        }));
    }
    handleMoveStart(x, y) {
        this.initialX = x;
        this.initialY = y;
        const rel = this.engine.getRelativePoint(x, y);
        this.initialXRelative = rel.x;
        this.initialYRelative = rel.y;
    }
    handleMove(x, y, event) {
        this.fireMouseMoved({
            displacementX: x - this.initialX,
            displacementY: y - this.initialY,
            virtualDisplacementX: (x - this.initialX) / (this.engine.getModel().getZoomLevel() / 100.0),
            virtualDisplacementY: (y - this.initialY) / (this.engine.getModel().getZoomLevel() / 100.0),
            event
        });
    }
    handleMoveEnd() {
        this.eject();
    }
}
//# sourceMappingURL=AbstractDisplacementState.js.map