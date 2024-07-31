import React from "react";
import {
  AbstractReactFactory,
  GenerateModelEvent,
  GenerateWidgetEvent,
} from "@projectstorm/react-canvas-core";
import { NodeModel } from "@projectstorm/react-diagrams";
import { TagNodeModel } from "./TagNodeModel";
import { TagNodeWidget } from "./TagNodeWidget";

export class TagNodeFactory extends AbstractReactFactory<NodeModel, any> {
  private callback: any;

  constructor(func: any) {
    super("tag");
    this.callback = func;
  }

  // Setup the generator method
  generateModel(event: GenerateModelEvent): NodeModel {
    return new TagNodeModel();
  }

  generateReactWidget(event: GenerateWidgetEvent<NodeModel>): JSX.Element {
    return (
      <div onDoubleClick={this.callback}>
        <TagNodeWidget engine={this.engine} node={event.model} />
      </div>
    );
  }
}
