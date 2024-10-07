import React, { useRef, memo , useState } from "react";
import createEngine, {
  DiagramModel,
  ZoomCanvasAction,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { configureEngine, changeColorNode} from "../helper/TreeEditorHelper";

const setTreeStatus = (model: any, engine:any, updateTree: any, baseTree: any) => {
  setStatusNode(model, engine, updateTree, baseTree);
};

const setStatusNode = (model: any, engine:any, updateTree: any, baseTree: any) => {
  var nodeName = baseTree["name"];
  var nodeId = baseTree["id"];

  var nodeChilds;
  try {
    nodeChilds = baseTree["childs"];
  } catch (error) {
    nodeChilds = [];
  }

  var nodeStatus = updateTree[nodeName]["state"];
  var node = model.getNode(nodeId);

  nodeChilds.forEach((element: any) => {
    setStatusNode(model, engine, updateTree[nodeName], element);
  });

  node.setExecStatus(nodeStatus);
  engine.repaintCanvas();
};

var a = 0;

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
    console.log("Diagram Visualizer " + a);
    a+=1;
    model.current.deserializeModel(modelJson, engine.current);
    model.current.setLocked(true);
    engine.current.setModel(model.current);

    const updateExecState = (msg: any) => {
      if (msg && msg.command === "update" && msg.data.update !== "") {
        const updateStatus = JSON.parse(msg.data.update);
        console.log("Repaint");
        const updateTree = updateStatus.tree;
        const updateBlackboard = updateStatus.blackboard;
        setTreeStatus(model.current, engine.current, updateTree, treeStructure);
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

export default DiagramVisualizer;
