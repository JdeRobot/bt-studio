import React, { useRef, memo, useState } from "react";
import createEngine, { DiagramModel } from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { changeColorNode, configureEngine } from "../helper/TreeEditorHelper";
import NodeMenu from "./NodeMenu";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";

const setTreeStatus = (
  model: any,
  engine: any,
  updateTree: any,
  baseTree: any,
  subtreeHierarchy: number[],
) => {
  var stateTree: any = updateTree;
  console.log(Object.values(stateTree));
  console.log("Base", baseTree);
  console.log("Hierarchy", subtreeHierarchy);

  if (subtreeHierarchy.length > 0) {
    subtreeHierarchy = subtreeHierarchy.concat([0]);
  }

  for (let index = 0; index < subtreeHierarchy.length; index++) {
    var moveTo = subtreeHierarchy[index];
    stateTree = Object.values(stateTree)[0];
    console.log(moveTo, stateTree);
    stateTree = Object.entries(stateTree)[moveTo + 1];
    var dict: any = {};
    const name = stateTree[0];
    dict[name] = stateTree[1];
    stateTree = dict;
  }

  console.log("Estate", stateTree);
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

  var nodeChilds;
  try {
    nodeChilds = baseTree["childs"];
  } catch (error) {
    nodeChilds = [];
  }

  console.trace(updateTree);
  var nodeStatus;
  try {
    nodeStatus = updateTree[nodeName]["state"];
  } catch (error) {
    nodeStatus = "NONE";
    if (updateTree) {
      var nodeData = Object.entries(updateTree)[index][1] as { state: string };
      nodeStatus = nodeData.state;
    }
  }

  var node = model.getNode(nodeId);

  var index = 1;
  // console.trace(nodeChilds)
  nodeChilds.forEach((element: any) => {
    console.trace(element);
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
    setSubTreeName,
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
    setSubTreeName: Function;
  }) => {
    // Initialize the model and the engine
    const model = useRef(new DiagramModel());
    const engine = useRef(createEngine());
    var lastClickedNodeId = "";

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

    // MODAL MANAGEMENT
    const openSubtree = () => {
      const node = model.current.getNode(lastClickedNodeId);
      lastClickedNodeId = "";
      model.current.clearSelection();
      if (node instanceof BasicNodeModel) {
        if (node.getIsSubtree()) {
          setSubTreeName(node.getName());
        }
      }
    };

    // Click listener
    const attachClickListener = (node: any) => {
      node.registerListener({
        selectionChanged: (event: any) => {
          if (event.isSelected) {
            lastClickedNodeId = node.getID();
          }
        },
      });
    };

    const zoomToFit = () => {
      engine.current.zoomToFitNodes({ margin: 50 });
    };

    // Configure the engine
    configureEngine(engine, openSubtree);

    // Deserialize and load the model
    console.log("Diagram Visualizer");
    model.current.deserializeModel(modelJson, engine.current);
    model.current.setLocked(true);
    engine.current.setModel(model.current);

    // After deserialization, attach listeners to each node
    const nodes = model.current.getNodes();
    nodes.forEach((node) => {
      attachClickListener(node);
    });

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
