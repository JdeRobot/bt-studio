import React from "react";
import { useRef, useMemo } from "react";

import createEngine, {
  DefaultLinkModel,
  DefaultNodeModel,
  DiagramModel,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { BasicNodeFactory } from "./nodes/basic_node/BasicNodeFactory";
import { TagNodeFactory } from "./nodes/tag_node/TagNodeFactory";
import { SimplePortFactory } from "./nodes/SimplePortFactory";
import { ChildrenPortModel } from "./nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "./nodes/basic_node/ports/parent_port/ParentPortModel";
import { OutputPortModel } from "./nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "./nodes/basic_node/ports/input_port/InputPortModel";
import { TagOutputPortModel } from "./nodes/tag_node/ports/output_port/TagOutputPortModel";
import { TagInputPortModel } from "./nodes/tag_node/ports/input_port/TagInputPortModel";

import NodeMenu from "./NodeMenu";

// Manages modals
const testFunction = () => {
  console.log("Hello!");
};

// Configures an engine with all the factories
const configureEngine = (engine: any) => {
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
      new SimplePortFactory("children", (config) => new ChildrenPortModel())
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("parent", (config) => new ParentPortModel())
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("output", (config) => new OutputPortModel(""))
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("input", (config) => new InputPortModel(""))
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("tag output", (config) => new TagOutputPortModel())
    );
  engine.current
    .getPortFactories()
    .registerFactory(
      new SimplePortFactory("tag input", (config) => new TagInputPortModel())
    );

  // Disable loose links
  const state: any = engine.current.getStateMachine().getCurrentState();
  state.dragNewLink.config.allowLooseLinks = false;
};

// Position listener
const attachPositionListener = (node: any) => {
  node.registerListener({
    positionChanged: (event: any) => {
      // lastMovedNodePosition = event.entity.getPosition();
      // setProjectChanges(true);
      // setModelJson(JSON.stringify(model.current.serialize())); // Serialize and update model JSON
    },
  });
};

// Click listener
const attachClickListener = (node: any) => {
  node.registerListener({
    selectionChanged: (event: any) => {
      if (event.isSelected) {
        // lastClickedNodeId.current = node.getID();
        node.selectNode();
      } else {
        node.deselectNode();
      }
    },
  });
};

const MinimalDiagramEditor = ({ modelJson }: { modelJson: any }) => {
  // Initialize the model and the engine
  const model = useRef(new DiagramModel());
  const engine = useRef(createEngine());
  configureEngine(engine);

  // Deserialize and load the model
  model.current.deserializeModel(modelJson, engine.current);
  engine.current.setModel(model.current);

  // After deserialization, attach listeners to each node
  const nodes = model.current.getNodes(); // Assuming getNodes() method exists to retrieve all nodes
  nodes.forEach((node) => {
    attachPositionListener(node);
    attachClickListener(node);
    node.setSelected(false);
  });

  return (
    <div>
      <NodeMenu />
      <CanvasWidget className="canvas" engine={engine.current} />
    </div>
  );
};

export default MinimalDiagramEditor;
