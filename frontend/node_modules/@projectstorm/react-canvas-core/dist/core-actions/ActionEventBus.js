import { InputType } from './Action';
import _filter from 'lodash/filter';
import _keys from 'lodash/keys';
export class ActionEventBus {
    constructor(engine) {
        this.actions = {};
        this.engine = engine;
        this.keys = {};
    }
    getKeys() {
        return _keys(this.keys);
    }
    registerAction(action) {
        action.setEngine(this.engine);
        this.actions[action.id] = action;
        return () => {
            this.deregisterAction(action);
        };
    }
    deregisterAction(action) {
        action.setEngine(null);
        delete this.actions[action.id];
    }
    getActionsForType(type) {
        return _filter(this.actions, (action) => {
            return action.options.type === type;
        });
    }
    getModelForEvent(actionEvent) {
        if (actionEvent.model) {
            return actionEvent.model;
        }
        return this.engine.getMouseElement(actionEvent.event);
    }
    getActionsForEvent(actionEvent) {
        const { event } = actionEvent;
        if (event.type === 'mousedown') {
            return this.getActionsForType(InputType.MOUSE_DOWN);
        }
        else if (event.type === 'mouseup') {
            return this.getActionsForType(InputType.MOUSE_UP);
        }
        else if (event.type === 'keydown') {
            // store the recorded key
            this.keys[event.key.toLowerCase()] = true;
            return this.getActionsForType(InputType.KEY_DOWN);
        }
        else if (event.type === 'keyup') {
            // delete the recorded key
            delete this.keys[event.key.toLowerCase()];
            return this.getActionsForType(InputType.KEY_UP);
        }
        else if (event.type === 'mousemove') {
            return this.getActionsForType(InputType.MOUSE_MOVE);
        }
        else if (event.type === 'wheel') {
            return this.getActionsForType(InputType.MOUSE_WHEEL);
        }
        else if (event.type === 'touchstart') {
            return this.getActionsForType(InputType.TOUCH_START);
        }
        else if (event.type === 'touchend') {
            return this.getActionsForType(InputType.TOUCH_END);
        }
        else if (event.type === 'touchmove') {
            return this.getActionsForType(InputType.TOUCH_MOVE);
        }
        return [];
    }
    fireAction(actionEvent) {
        const actions = this.getActionsForEvent(actionEvent);
        for (let action of actions) {
            action.options.fire(actionEvent);
        }
    }
}
//# sourceMappingURL=ActionEventBus.js.map