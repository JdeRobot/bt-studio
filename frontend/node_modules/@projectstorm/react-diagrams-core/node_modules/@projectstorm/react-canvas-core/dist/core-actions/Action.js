import { Toolkit } from '../Toolkit';
export var InputType;
(function (InputType) {
    InputType["MOUSE_DOWN"] = "mouse-down";
    InputType["MOUSE_UP"] = "mouse-up";
    InputType["MOUSE_MOVE"] = "mouse-move";
    InputType["MOUSE_WHEEL"] = "mouse-wheel";
    InputType["KEY_DOWN"] = "key-down";
    InputType["KEY_UP"] = "key-up";
    InputType["TOUCH_START"] = "touch-start";
    InputType["TOUCH_END"] = "touch-end";
    InputType["TOUCH_MOVE"] = "touch-move";
})(InputType || (InputType = {}));
export class Action {
    constructor(options) {
        this.options = options;
        this.id = Toolkit.UID();
    }
    setEngine(engine) {
        this.engine = engine;
    }
}
//# sourceMappingURL=Action.js.map