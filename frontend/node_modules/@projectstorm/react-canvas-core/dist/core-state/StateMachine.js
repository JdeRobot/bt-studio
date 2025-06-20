import _last from 'lodash/last';
import { BaseObserver } from '../core/BaseObserver';
export class StateMachine extends BaseObserver {
    constructor(engine) {
        super();
        this.engine = engine;
        this.stateStack = [];
    }
    getCurrentState() {
        return this.currentState;
    }
    pushState(state) {
        this.stateStack.push(state);
        this.setState(state);
    }
    popState() {
        this.stateStack.pop();
        this.setState(_last(this.stateStack));
    }
    setState(state) {
        state.setEngine(this.engine);
        // if no state object, get the initial state
        if (this.currentState) {
            this.currentState.deactivated(state);
        }
        const old = this.currentState;
        this.currentState = state;
        if (this.currentState) {
            this.currentState.activated(old);
            this.fireEvent({
                newState: state
            }, 'stateChanged');
        }
    }
}
//# sourceMappingURL=StateMachine.js.map