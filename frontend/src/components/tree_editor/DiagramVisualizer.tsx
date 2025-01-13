import { useRef, memo } from "react";
import createEngine, {
  DiagramEngine,
  DiagramModel,
  NodeModel,
  NodeModelGenerics,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import {
  changeColorNode,
  configureEngine,
  TreeViewType,
} from "../helper/TreeEditorHelper";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";

import "./TreeEditor.css";
import NodeMenuMinimal from "./NodeMenuMinimal";
import CommsManager from "../../api_helper/CommsManager";

const setTreeStatus = (
  model: DiagramModel,
  engine: DiagramEngine,
  updateTree: any,
  baseTree: any,
  subtreeHierarchy: number[],
) => {
  var stateTree: any = updateTree;
  console.log(Object.values(stateTree));
  console.log("Base", baseTree);
  console.log("Hierarchy", subtreeHierarchy);

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
  model: DiagramModel,
  engine: DiagramEngine,
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

  var nodeStatus;
  try {
    nodeStatus = updateTree[nodeName]["state"];
  } catch (error) {
    nodeStatus = "NONE";
    //FIX: bug here
    if (updateTree) {
      var nodeData = Object.entries(updateTree)[index][1] as { state: string };
      nodeStatus = nodeData.state;
    }
  }

  var node = model.getNode(nodeId) as BasicNodeModel;

  var newIndex = 1;
  // console.trace(nodeChilds)
  nodeChilds.forEach((element: any) => {
    setStatusNode(model, engine, updateTree[nodeName], element, newIndex);
    newIndex += 1;
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
    changeColorNode(rgb, undefined, node, engine, model);
  }
};

const updateBlackboardValues = (
  model: DiagramModel,
  engine: DiagramEngine,
  blackboard: any,
) => {
  const blackboardRegex = /^\{[^}]*\}/i;
  let tags = model.getNodes().filter(function (node) {
    return node instanceof TagNodeModel && blackboardRegex.test(node.getName());
  });

  console.log(tags);
  let notFoundTags: NodeModel<NodeModelGenerics>[] = [];

  Object.entries(blackboard).forEach((element) => {
    console.log(element);
    for (let index = 0; index < tags.length; index++) {
      const tag = tags[index] as TagNodeModel;
      let tagSplit = tag.getName().split(" = ");
      const tagStr = tagSplit[0].slice(1, -1); // Remove {}
      if (tagStr === element[0]) {
        tag.setName(`{${tagStr}} = ${element[1]}`);
      } else {
        notFoundTags.push(tag);
      }
    }
    tags = notFoundTags;
    notFoundTags = [];
  });

  engine.repaintCanvas();
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
    manager: CommsManager;
    treeStructure: any;
    view: TreeViewType;
    changeView: Function;
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
      // TODO: add some kind of limit of updates per second to avoid crashes
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
        updateBlackboardValues(model.current, engine.current, updateBlackboard);
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
    const attachClickListener = (node: NodeModel) => {
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
      <>
        <NodeMenuMinimal
          onZoomToFit={zoomToFit}
          view={view}
          changeView={changeView}
          setGoBack={setGoBack}
          subTreeName={subTreeName}
        />
        {engine.current && (
          <CanvasWidget className="bt-canvas" engine={engine.current} />
        )}
      </>
    );
  },
);

export default DiagramVisualizer;
