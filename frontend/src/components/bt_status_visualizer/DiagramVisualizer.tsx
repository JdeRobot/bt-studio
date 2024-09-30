import React, { useCallback, useEffect, useReducer, useState } from "react";
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

const setTreeStatus = (model: any, updateTree: any, baseTree: any) => {
  console.log(updateTree);

  setStatusNode(model, updateTree, baseTree);
};

const setStatusNode = (model: any, updateTree: any, baseTree: any) => {
  var nodeName = baseTree["name"];
  var nodeId = baseTree["id"];

  var nodeChilds;
  try {
    nodeChilds = baseTree["childs"];
  } catch (error) {
    nodeChilds = [];
  }

  console.log(updateTree[nodeName], nodeName);
  var nodeStatus = updateTree[nodeName]["state"];
  var node = model.current.getNode(nodeId);

  nodeChilds.forEach((element: any) => {
    setStatusNode(model, updateTree[nodeName], element);
  });
  node.setExecStatus(nodeStatus);
  model.current.addNode(node);
};

const DiagramVisualizer = memo(
  ({
    modelJson,
    manager,
    treeStructure,
  }: {
    modelJson: any;
    manager: any;
    treeStructure: any;
  }) => {
    // Initialize the model and the engine
    const model = useRef(new DiagramModel());
    const engine = useRef(createEngine());

    // There is no need to use an effect as the editor will re render when the model json changes
    // Configure the engine
    configureEngine(engine);

    // Deserialize and load the model
    console.log("Diagram Visualizer");
    model.current.deserializeModel(modelJson, engine.current);
    model.current.setLocked(true);
    engine.current.setModel(model.current);

    const updateExecState = (msg: any) => {
      if (msg && msg.command === "update" && msg.data.update !== "") {
        const updateStatus = JSON.parse(msg.data.update);
        console.log("Repaint");
        const updateTree = updateStatus.tree;
        const updateBlackboard = updateStatus.blackboard;

        setTreeStatus(model, updateTree, treeStructure);
        engine.current.repaintCanvas();
      }
    };

    manager.subscribe("update", updateExecState);

    return (
      <div>
        <CanvasWidget className="canvas" engine={engine.current} />
      </div>
    );
  },
);

const DiagramVisualizerStatus = ({
  model,
  engine,
  manager,
  treeStructure,
}: {
  model: any;
  engine: any;
  manager: any;
  treeStructure: any;
}) => {
  const [s, st] = useState("");

  const updateExecState = (msg: any) => {
    if (msg && msg.command === "update" && msg.data.update !== "") {
      const updateStatus = JSON.parse(msg.data.update);
      console.log("Repaint");
      const updateTree = updateStatus.tree;
      const updateBlackboard = updateStatus.blackboard;

      setTreeStatus(model, updateTree, treeStructure);
      engine.current.repaintCanvas();
    }
  };

  manager.subscribe("update", updateExecState);

  return (
    <div>
      <CanvasWidget className="canvas" engine={engine.current} />
    </div>
  );
};

export default DiagramVisualizer;
