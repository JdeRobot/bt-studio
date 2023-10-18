import * as React from 'react';
import { DefaultNodeModel } from './DefaultNodeModel';
import { DefaultNodeWidget } from './DefaultNodeWidget';
import { AbstractReactFactory } from '@projectstorm/react-canvas-core';
export class DefaultNodeFactory extends AbstractReactFactory {
    constructor() {
        super('default');
    }
    generateReactWidget(event) {
        return React.createElement(DefaultNodeWidget, { engine: this.engine, node: event.model });
    }
    generateModel(event) {
        return new DefaultNodeModel();
    }
}
//# sourceMappingURL=DefaultNodeFactory.js.map