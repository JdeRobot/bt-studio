import React, { useEffect, useState } from "react";
import { useRef, memo } from "react";

import createEngine, {
  DiagramModel,
  ZoomCanvasAction,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { BasicNodeFactory } from "../diagram_editor/nodes/basic_node/BasicNodeFactory";
import { TagNodeFactory } from "../diagram_editor/nodes/tag_node/TagNodeFactory";
import { SimplePortFactory } from "../diagram_editor/nodes/SimplePortFactory";
import { ChildrenPortModel } from "../diagram_editor/nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "../diagram_editor/nodes/basic_node/ports/parent_port/ParentPortModel";
import { OutputPortModel } from "../diagram_editor/nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "../diagram_editor/nodes/basic_node/ports/input_port/InputPortModel";
import { TagOutputPortModel } from "../diagram_editor/nodes/tag_node/ports/output_port/TagOutputPortModel";
import { TagInputPortModel } from "../diagram_editor/nodes/tag_node/ports/input_port/TagInputPortModel";


// MODAL MANAGEMENT
const testFunction = () => {
  console.log("Hello!");
};

// HELPERS

// Configures an engine with all the factories
const configureEngine = (engine: any) => {
  console.log("Configuring engine!");
  // Register factories
  engine.current
    .getNodeFactories()
    .registerFactory(new BasicNodeFactory(testFunction));
  engine.current
    .getNodeFactories()
    .registerFactory(new TagNodeFactory(testFunction));
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("children", (config) => new ChildrenPortModel()),
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("parent", (config) => new ParentPortModel()),
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("output", (config) => new OutputPortModel("")),
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("input", (config) => new InputPortModel("")),
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("tag output", (config) => new TagOutputPortModel()),
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("tag input", (config) => new TagInputPortModel()),
    );

  // Disable loose links
  const state: any = engine.current.getStateMachine().getCurrentState();
  state.dragNewLink.config.allowLooseLinks = false;

  engine.current
      .getActionEventBus()
      .registerAction(new ZoomCanvasAction({ inverseZoom: true }));
};

const DiagramVisualizer = memo(
  ({
    modelJson,
    projectName
  }: {
    modelJson: any;
    projectName: string;
  }) => {

    // Initialize the model and the engine
    const model = useRef(new DiagramModel());
    const engine = useRef(createEngine());

    // There is no need to use an effect as the editor will re render when the model json changes
    // Configure the engine
    configureEngine(engine);

    // Deserialize and load the model
    model.current.deserializeModel(modelJson, engine.current);
    model.current.setLocked(true);
    engine.current.setModel(model.current);

    return (
      <div>
        <CanvasWidget className="canvas" engine={engine.current} />
      </div>
    );
  },
);

export default DiagramVisualizer;