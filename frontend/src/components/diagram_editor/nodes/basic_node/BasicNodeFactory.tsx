import React from 'react';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { NodeModel } from '@projectstorm/react-diagrams';
import { BasicNodeModel } from './BasicNodeModel';
import { BasicNodeWidget } from './BasicNodeWidget';

export class BasicNodeFactory extends AbstractReactFactory<NodeModel, any> {

    constructor() {
        super('basic');
    }

    // Setup the generator method
    generateModel(event: GenerateModelEvent): NodeModel {
        return new BasicNodeModel();
    }

    generateReactWidget(event: GenerateWidgetEvent<NodeModel>): JSX.Element {
        return <BasicNodeWidget engine={this.engine} node={event.model} />;
    }
}
