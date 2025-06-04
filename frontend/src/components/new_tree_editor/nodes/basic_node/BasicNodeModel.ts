import {
  NodeModel,
  NodeModelGenerics,
} from "@projectstorm/react-diagrams";
import { ParentPortModel } from "./ports/parent_port/ParentPortModel";
import { ChildrenPortModel } from "./ports/children_port/ChildrenPortModel";
import { DeserializeEvent } from "@projectstorm/react-canvas-core";
import { InputPortModel } from "./ports/input_port/InputPortModel";
import { OutputPortModel } from "./ports/output_port/OutputPortModel";

export interface BasicNodeModelGenerics {
  PORT: ParentPortModel | ChildrenPortModel | InputPortModel | OutputPortModel;
}

export type BTExecutionStatus =
  | "RUNNING"
  | "SUCCESS"
  | "FAILURE"
  | "INVALID"
  | "NONE";

export class BasicNodeModel extends NodeModel<
  NodeModelGenerics & BasicNodeModelGenerics
> {
  private name: string;
  private color: string;
  private is_selected: boolean;
  private is_subtree: boolean;
  private exec_status: BTExecutionStatus;

  constructor(
    name: string = "Basic Node",
    color: string = "rgb(0,192,255)",
    is_subtree: boolean = false,
  ) {
    super({
      type: "basic",
    });
    this.name = name;
    this.color = color;
    this.is_selected = false;
    this.is_subtree = is_subtree;
    this.exec_status = "NONE";
  }

  getName(): string {
    return this.name;
  }

  getIsSubtree(): boolean {
    return this.is_subtree;
  }

  setColor(color: string): void {
    this.color = color;
  }

  getColor(): string {
    return this.color;
  }

  setExecStatus(status: BTExecutionStatus): void {
    this.exec_status = status;
  }

  getExecStatus(): BTExecutionStatus {
    return this.exec_status;
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
      is_selected: this.is_selected,
      is_subtree: this.is_subtree,
    };
  }

  deserialize(event: DeserializeEvent<this>): void {
    super.deserialize(event);
    this.name = event.data.name;
    this.color = event.data.color;
    this.is_selected = event.data.is_selected;
    this.is_subtree = event.data.is_subtree;
  }
}
