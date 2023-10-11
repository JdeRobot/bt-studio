import * as React from 'react';
import { AbstractReactFactory } from '@projectstorm/react-canvas-core';
import { NodeLayerModel } from './NodeLayerModel';
import { NodeLayerWidget } from './NodeLayerWidget';
export class NodeLayerFactory extends AbstractReactFactory {
    constructor() {
        super('diagram-nodes');
    }
    generateModel(event) {
        return new NodeLayerModel();
    }
    generateReactWidget(event) {
        return React.createElement(NodeLayerWidget, { layer: event.model, engine: this.engine });
    }
}
//# sourceMappingURL=NodeLayerFactory.js.map