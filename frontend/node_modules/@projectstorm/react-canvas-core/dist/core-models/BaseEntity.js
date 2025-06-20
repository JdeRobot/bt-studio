import { Toolkit } from '../Toolkit';
import _cloneDeep from 'lodash/cloneDeep';
import { BaseObserver } from '../core/BaseObserver';
export class BaseEntity extends BaseObserver {
    constructor(options = {}) {
        super();
        this.options = Object.assign({ id: Toolkit.UID() }, options);
    }
    getOptions() {
        return this.options;
    }
    getID() {
        return this.options.id;
    }
    doClone(lookupTable = {}, clone) {
        /*noop*/
    }
    clone(lookupTable = {}) {
        // try and use an existing clone first
        if (lookupTable[this.options.id]) {
            return lookupTable[this.options.id];
        }
        let clone = _cloneDeep(this);
        clone.options = Object.assign(Object.assign({}, this.options), { id: Toolkit.UID() });
        clone.clearListeners();
        lookupTable[this.options.id] = clone;
        this.doClone(lookupTable, clone);
        return clone;
    }
    clearListeners() {
        this.listeners = {};
    }
    deserialize(event) {
        this.options.id = event.data.id;
        this.options.locked = event.data.locked;
    }
    serialize() {
        return {
            id: this.options.id,
            locked: this.options.locked
        };
    }
    fireEvent(event, k) {
        super.fireEvent(Object.assign({ entity: this }, event), k);
    }
    isLocked() {
        return this.options.locked;
    }
    setLocked(locked = true) {
        this.options.locked = locked;
        this.fireEvent({
            locked: locked
        }, 'lockChanged');
    }
}
//# sourceMappingURL=BaseEntity.js.map