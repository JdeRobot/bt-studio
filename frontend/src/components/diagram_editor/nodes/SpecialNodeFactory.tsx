import React from 'react';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { NodeModel } from '@projectstorm/react-diagrams';
import { SpecialNodeModel } from './SpecialNodeModel';
import { SpecialNodeWidget } from './SpecialNodeWidget';

export class SpecialNodeFactory extends AbstractReactFactory<NodeModel, any> {

    constructor() {
        super('special');
    }

    // Setup the generator method
    generateModel(event: GenerateModelEvent): NodeModel {
        return new SpecialNodeModel();
    }

    generateReactWidget(event: GenerateWidgetEvent<NodeModel>): JSX.Element {
        return <SpecialNodeWidget engine={this.engine} node={event.model} />;
    }
}
