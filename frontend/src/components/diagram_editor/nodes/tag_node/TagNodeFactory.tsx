import React from 'react';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { NodeModel } from '@projectstorm/react-diagrams';
import { TagNodeModel } from './TagNodeModel';
import { TagNodeWidget } from './TagNodeWidget';

export class TagNodeFactory extends AbstractReactFactory<NodeModel, any> {

    constructor() {
        super('tag');
    }

    // Setup the generator method
    generateModel(event: GenerateModelEvent): NodeModel {
        return new TagNodeModel();
    }

    generateReactWidget(event: GenerateWidgetEvent<NodeModel>): JSX.Element {
        return <TagNodeWidget engine={this.engine} node={event.model} />;
    }
}
