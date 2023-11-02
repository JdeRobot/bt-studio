import { DefaultNodeModel, DefaultPortModel, NodeModel, NodeModelGenerics, PortModelAlignment } from '@projectstorm/react-diagrams';
import { DeserializeEvent } from '@projectstorm/react-canvas-core';
import { TagInputPortModel } from './ports/input_port/TagInputPortModel';
import { TagOutputPortModel } from './ports/output_port/TagOutputPortModel';

export interface TagNodeModelGenerics {
    PORT: TagInputPortModel | TagOutputPortModel;
}

export class TagNodeModel extends NodeModel<NodeModelGenerics & TagNodeModelGenerics> {

    private name: string;
    private color: string;

    constructor(name: string = 'Tag Node', color: string = 'rgb(0,192,255)') {
        super({
            type: "tag",
        });
        this.name = name;
        this.color = color;
    }

    getName(): string {
        return this.name;
    }

    setName(newName : string) {
        this.name = newName;
    }

    getColor(): string {
        return this.color;
    }

    // Method to add children port (they can be default model because only the widget, the visualization changes)
    addInputPort() {
        const port = new TagInputPortModel();
        this.addPort(port);
        return port;
    }

    addOutputPort() {
        const port = new TagOutputPortModel();
        this.addPort(port);
        return port;
    }

    serialize() {
        return {
          ...super.serialize(),
          name: this.name,
          color: this.color
        };
    }

    deserialize(event: DeserializeEvent<this>): void {
        super.deserialize(event);
        this.name = event.data.name;
        this.color = event.data.color;
    }
}
