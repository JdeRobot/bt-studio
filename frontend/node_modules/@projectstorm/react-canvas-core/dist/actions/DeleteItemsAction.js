import { Action, InputType } from '../core-actions/Action';
import _forEach from 'lodash/forEach';
import _isEqual from 'lodash/isEqual';
/**
 * Deletes all selected items
 */
export class DeleteItemsAction extends Action {
    constructor(options = {}) {
        const keyCodes = options.keyCodes || [46, 8];
        const modifiers = Object.assign({ ctrlKey: false, shiftKey: false, altKey: false, metaKey: false }, options.modifiers);
        super({
            type: InputType.KEY_DOWN,
            fire: (event) => {
                const { keyCode, ctrlKey, shiftKey, altKey, metaKey } = event.event;
                if (keyCodes.indexOf(keyCode) !== -1 && _isEqual({ ctrlKey, shiftKey, altKey, metaKey }, modifiers)) {
                    _forEach(this.engine.getModel().getSelectedEntities(), (model) => {
                        // only delete items which are not locked
                        if (!model.isLocked()) {
                            model.remove();
                        }
                    });
                    this.engine.repaintCanvas();
                }
            }
        });
    }
}
//# sourceMappingURL=DeleteItemsAction.js.map