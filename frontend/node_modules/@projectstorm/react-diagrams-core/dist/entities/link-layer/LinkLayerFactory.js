import * as React from 'react';
import { AbstractReactFactory } from '@projectstorm/react-canvas-core';
import { LinkLayerModel } from './LinkLayerModel';
import { LinkLayerWidget } from './LinkLayerWidget';
export class LinkLayerFactory extends AbstractReactFactory {
    constructor() {
        super('diagram-links');
    }
    generateModel(event) {
        return new LinkLayerModel();
    }
    generateReactWidget(event) {
        return React.createElement(LinkLayerWidget, { layer: event.model, engine: this.engine });
    }
}
//# sourceMappingURL=LinkLayerFactory.js.map