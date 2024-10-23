import React, { useRef, memo, useState } from "react";
import createEngine, { DiagramModel } from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { changeColorNode, configureEngine } from "../helper/TreeEditorHelper";
import NodeMenu from "./NodeMenu";

const setTreeStatus = (
  model: any,
  engine: any,
  updateTree: any,
  baseTree: any,
  subtreeHierarchy: number[],
) => {
  var stateTree: any = updateTree;

  for (let index = 0; index < subtreeHierarchy.length; index++) {
    var moveTo = subtreeHierarchy[index];
    stateTree = Object.values(stateTree)[0];
    stateTree = Object.entries(stateTree)[moveTo + 1];
    var dict: any = {};
    const name = stateTree[0];
    dict[name] = stateTree[1];
    stateTree = dict;
  }

  // console.log(updateTree);
  console.log("Estate", stateTree);
  console.log("Base", baseTree);
  console.log("Hierarchy", subtreeHierarchy);
  setStatusNode(model, engine, stateTree, baseTree);
};

const setStatusNode = (
  model: any,
  engine: any,
  updateTree: any,
  baseTree: any,
  index: number = 0,
) => {
  var nodeName = baseTree["name"];
  var nodeId = baseTree["id"];

  console.log(nodeName, nodeId);

  var nodeChilds;
  try {
    nodeChilds = baseTree["childs"];
  } catch (error) {
    nodeChilds = [];
  }

  //TODO: fix for decorators

  var nodeStatus;
  try {
    nodeStatus = updateTree[nodeName]["state"];
  } catch (error) {
    nodeStatus = "NONE";
    if (updateTree) {
      console.log(updateTree);
      var nodeData = Object.entries(updateTree)[index][1] as { state: string };
      nodeStatus = nodeData.state;
    }
  }

  var node = model.getNode(nodeId);

  var index = 1;
  nodeChilds.forEach((element: any) => {
    setStatusNode(model, engine, updateTree[nodeName], element, index);
    index += 1;
  });

  // node.setExecStatus(nodeStatus);
  // engine.repaintCanvas();
  var rgb: [number, number, number] = [100, 100, 100];

  switch (nodeStatus) {
    case "RUNNING":
      rgb = [255, 150, 0];
      break;
    case "SUCCESS":
      rgb = [0, 250, 0];
      break;
    case "FAILURE":
      rgb = [200, 0, 0];
      break;
    default:
      rgb = [100, 100, 100];
      break;
  }

  if (node) {
    changeColorNode(rgb, node, engine, model);
  }
};

const DiagramVisualizer = memo(
  ({
    modelJson,
    setResultJson,
    manager,
    treeStructure,
    view,
    changeView,
    setGoBack,
    subTreeName,
    subTreeStructure,
  }: {
    modelJson: any;
    setResultJson: Function;
    manager: any;
    treeStructure: any;
    view: any;
    changeView: any;
    setGoBack: Function;
    subTreeName: string;
    subTreeStructure: number[];
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
        setTreeStatus(
          model.current,
          engine.current,
          updateTree,
          treeStructure,
          subTreeStructure,
        );
      }
    };

    manager.unsubscribe("update", updateExecState);
    manager.subscribe("update", updateExecState);

    const zoomToFit = () => {
      engine.current.zoomToFitNodes({ margin: 50 });
    };

    // Fixes uncomplete first serialization
    setTimeout(() => {
      console.log("Rendered!");
      setResultJson(model.current.serialize());
    }, 1);

    return (
      <div>
        <NodeMenu
          projectName={"projectName"}
          onAddNode={() => {}}
          onDeleteNode={() => {}}
          onZoomToFit={zoomToFit}
          onEditAction={() => {}}
          hasSubtrees={false}
          view={view}
          changeView={changeView}
          setGoBack={setGoBack}
          subTreeName={subTreeName}
        />
        {engine.current && (
          <CanvasWidget className="canvas" engine={engine.current} />
        )}
      </div>
    );
  },
);

export default DiagramVisualizer;
