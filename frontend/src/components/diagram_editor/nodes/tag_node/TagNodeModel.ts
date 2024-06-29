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
    private is_blackboard: boolean;
    private is_selected: boolean;

    private static select_border_color: string = 'rgb(18,109,114)';
    private static base_border_color: string = 'rgb(0,0,0)';

    private static select_border_radius: string = '5';
    private static base_border_radius: string = '1';

    constructor(name: string = 'Tag Node', color: string = 'rgb(128,128,128)') {
        super({
            type: "tag",
        });
        this.name = name;
        this.color = color;
        this.is_blackboard = false;
        this.is_selected = false;
    }

    getName(): string {
        return this.name;
    }

    setName(newName : string) {
        this.name = newName;
        if (this.name.match(/\{([^)]+)\}/)) {
            this.is_blackboard = true;
        } else {
            this.is_blackboard = false;
        }
    }

    isFromBlackboard() {
        return this.is_blackboard;
    }

    getColor(): string {
        return this.color;
    }

    getBorderColor(): string {
        if (this.is_selected) {return TagNodeModel.select_border_color;}
        else { return TagNodeModel.base_border_color;}
    }

    getBorderRadius(): string {
        if (this.is_selected) {return TagNodeModel.select_border_radius;}
        else { return TagNodeModel.base_border_radius;}
    }

    selectNode() {
        this.is_selected = true;
    }

    deselectNode() {
        this.is_selected = false;
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
