import * as React from 'react';
import { AbstractReactFactory } from '../../core/AbstractReactFactory';
import { SelectionLayerModel } from './SelectionLayerModel';
import { SelectionBoxWidget } from './SelectionBoxWidget';
export class SelectionBoxLayerFactory extends AbstractReactFactory {
    constructor() {
        super('selection');
    }
    generateModel(event) {
        return new SelectionLayerModel();
    }
    generateReactWidget(event) {
        return React.createElement(SelectionBoxWidget, { rect: event.model.box });
    }
}
//# sourceMappingURL=SelectionBoxLayerFactory.js.map