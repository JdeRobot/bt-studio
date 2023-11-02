import { DefaultNodeModel, DefaultPortModel, NodeModel, NodeModelGenerics, PortModelAlignment } from '@projectstorm/react-diagrams';
import { ParentPortModel } from './ports/parent_port/ParentPortModel';
import { ChildrenPortModel } from './ports/children_port/ChildrenPortModel';
import { DeserializeEvent } from '@projectstorm/react-canvas-core';
import { InputPortModel } from './ports/input_port/InputPortModel';
import { OutputPortModel } from './ports/output_port/OutputPortModel';

export interface BasicNodeModelGenerics {
    PORT: ParentPortModel | ChildrenPortModel | InputPortModel | OutputPortModel;
}

export class BasicNodeModel extends NodeModel<NodeModelGenerics & BasicNodeModelGenerics> {

    private name: string;
    private color: string;

    constructor(name: string = 'Basic Node', color: string = 'rgb(0,192,255)') {
        super({
            type: "basic",
        });
        this.name = name;
        this.color = color;
    }

    getName(): string {
        return this.name;
    }

    getColor(): string {
        return this.color;
    }

    addChildrenPort(name: string) {
        const port = new ChildrenPortModel();
        this.addPort(port);
        return port;
    }
    
    addParentPort(name: string) {
        const port = new ParentPortModel();
        this.addPort(port);
        return port;
    }

    addInputPort(name: string) {
        const port = new InputPortModel(name);
        this.addPort(port);
        return port;
    }

    addOutputPort(name: string) {
        const port = new OutputPortModel(name);
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
