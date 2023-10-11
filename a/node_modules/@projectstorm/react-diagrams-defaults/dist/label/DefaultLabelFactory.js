import * as React from 'react';
import { DefaultLabelModel } from './DefaultLabelModel';
import { DefaultLabelWidget } from './DefaultLabelWidget';
import { AbstractReactFactory } from '@projectstorm/react-canvas-core';
/**
 * @author Dylan Vorster
 */
export class DefaultLabelFactory extends AbstractReactFactory {
    constructor() {
        super('default');
    }
    generateReactWidget(event) {
        return React.createElement(DefaultLabelWidget, { model: event.model });
    }
    generateModel(event) {
        return new DefaultLabelModel();
    }
}
//# sourceMappingURL=DefaultLabelFactory.js.map