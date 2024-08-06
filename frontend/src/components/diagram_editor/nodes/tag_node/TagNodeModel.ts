import {
  DefaultNodeModel,
  DefaultPortModel,
  NodeModel,
  NodeModelGenerics,
  PortModelAlignment,
} from "@projectstorm/react-diagrams";
import { DeserializeEvent } from "@projectstorm/react-canvas-core";
import { TagInputPortModel } from "./ports/input_port/TagInputPortModel";
import { TagOutputPortModel } from "./ports/output_port/TagOutputPortModel";

export interface TagNodeModelGenerics {
  PORT: TagInputPortModel | TagOutputPortModel;
}

export class TagNodeModel extends NodeModel<
  NodeModelGenerics & TagNodeModelGenerics
> {
  private name: string;
  private color: string;
  private is_blackboard: boolean;
  private is_selected: boolean;

  private static blackboard_color: string = "#5BA498";
  private static value_color: string = "#A45B67";

  constructor(name: string = "Tag Node", color: string = "rgb(128,128,128)") {
    super({
      type: "tag",
    });
    console.log(name);
    this.name = name;
    this.color = color;
    this.is_blackboard = false;
    this.is_selected = false;
  }

  getName(): string {
    return this.name;
  }

  setName(newName: string) {
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
    return this.is_blackboard
      ? TagNodeModel.blackboard_color
      : TagNodeModel.value_color;
  }

  isSelected(): boolean {
    return this.is_selected;
  }

  selectNode() {
    this.is_selected = true;
    this.setSelected(true);
  }

  deselectNode() {
    this.is_selected = false;
    this.setSelected(false);
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
      color: this.getColor(),
      is_selected: this.is_selected,
      is_blackboard: this.is_blackboard,
    };
  }

  deserialize(event: DeserializeEvent<this>): void {
    super.deserialize(event);
    this.name = event.data.name;
    this.color = event.data.color;
    this.is_selected = event.data.is_selected;
    this.is_blackboard = event.data.is_blackboard;
  }
}
