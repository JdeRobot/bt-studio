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
    private is_selected: boolean;

    private static select_border_color: string = 'rgb(18,109,114)';
    private static base_border_color: string = 'rgb(0,0,0)';

    private static select_border_radius: string = '5';
    private static base_border_radius: string = '1';


    constructor(name: string = 'Basic Node', color: string = 'rgb(0,192,255)') {
        super({
            type: "basic",
        });
        this.name = name;
        this.color = color;
        this.is_selected = false;
    }

    getName(): string {
        return this.name;
    }

    setColor(color: string): void {
        this.color = color;
    }

    getColor(): string {
        return this.color;
    }

    getBorderColor(): string {
        if (this.is_selected) {return BasicNodeModel.select_border_color;}
        else { return BasicNodeModel.base_border_color;}
    }

    getBorderRadius(): string {
        if (this.is_selected) {return BasicNodeModel.select_border_radius;}
        else { return BasicNodeModel.base_border_radius;}
    }

    selectNode() {
        this.is_selected = true;
    }

    deselectNode() {
        this.is_selected = false;
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

    removeInputPort(port: InputPortModel) {
        this.removePort(port);
    }

    addOutputPort(name: string) {
        const port = new OutputPortModel(name);
        this.addPort(port);
        return port;
    }

    removeOutputPort(port: OutputPortModel) {
        this.removePort(port);
    }

    serialize() {
        return {
          ...super.serialize(),
          name: this.name,
          color: this.color,
          is_selected: this.is_selected
        };
    }

    deserialize(event: DeserializeEvent<this>): void {
        super.deserialize(event);
        this.name = event.data.name;
        this.color = event.data.color;
        this.is_selected = event.data.is_selected;
    }
}
