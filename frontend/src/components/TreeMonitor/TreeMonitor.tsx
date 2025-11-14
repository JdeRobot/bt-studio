import React from "react";
import { useRef, MutableRefObject } from "react";
import createEngine, {
  DiagramEngine,
  DiagramModel,
  NodeModel,
  NodeModelGenerics,
} from "@projectstorm/react-diagrams";
import { useBtTheme } from "BtContexts/BtThemeContext";

import { changeColorNode, configureEngine } from "../helper/TreeEditorHelper";
import { BasicNodeModel } from "../TreeEditor/nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "../TreeEditor/nodes/tag_node/TagNodeModel";
import TreeMonitorMenu from "./TreeMonitorMenu";
import { CommsManager } from "jderobot-commsmanager";
import { StyledBTCanvas } from "BtStyles/TreeEditor/BTCanvas.styles";

const setTreeStatus = (
  model: DiagramModel,
  engine: DiagramEngine,
  updateTree: any,
  baseTree: any,
  subtreeHierarchy: number[],
) => {
  let stateTree: any = updateTree;
  console.log(Object.values(stateTree));
  console.log("Base", baseTree);
  console.log("Hierarchy", subtreeHierarchy);

  for (let index = 0; index < subtreeHierarchy.length; index++) {
    const moveTo = subtreeHierarchy[index];
    stateTree = Object.values(stateTree)[0];
    stateTree = [stateTree["childs"][moveTo]];
  }

  console.log("State", stateTree);
  setStatusNode(model, engine, stateTree, baseTree);
};

const setStatusNode = (
  model: DiagramModel,
  engine: DiagramEngine,
  updateTree: any,
  baseTree: any,
  index: number = 0,
) => {
  const nodeId = baseTree["id"];

  let nodeChilds;
  try {
    nodeChilds = baseTree["childs"];
  } catch (error) {
    nodeChilds = [];
  }

  let nodeStatus;
  try {
    nodeStatus = updateTree[index]["state"];
  } catch (error) {
    nodeStatus = "NONE";
    //FIX: bug here
    if (updateTree) {
      const nodeData = Object.entries(updateTree)[index][1] as {
        state: string;
      };
      nodeStatus = nodeData.state;
    }
  }

  const node = model.getNode(nodeId) as BasicNodeModel;

  let newIndex = 0;
  nodeChilds.forEach((element: any) => {
    setStatusNode(
      model,
      engine,
      updateTree[index]["childs"],
      element,
      newIndex,
    );
    newIndex += 1;
  });

  // node.setExecStatus(nodeStatus);
  // engine.repaintCanvas();
  let rgb: [number, number, number] = [100, 100, 100];

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
      const tagSplit = tag.getName().split(" = ");
      const tagStr = tagSplit[0].slice(1, -1); // Remove {}
      if (tagStr === element[0]) {
        tag.setName(`{${tagStr}} = ${(element[1] as string).slice(0, 50)}`);
      } else {
        notFoundTags.push(tag);
      }
    }
    tags = notFoundTags;
    notFoundTags = [];
  });

  engine.repaintCanvas();
};

const TreeMonitor = ({
  modelJson,
  setResultJson,
  manager,
  treeStructure,
  setGoBack,
  subTreeName,
  subTreeStructure,
  enterSubtree,
  render,
}: {
  modelJson: any;
  setResultJson: (data: string) => void;
  manager: CommsManager;
  treeStructure: any;
  setGoBack: (a: boolean) => void;
  subTreeName: MutableRefObject<string>;
  subTreeStructure: number[];
  enterSubtree: (name?: string) => Promise<void>;
  render: MutableRefObject<boolean>;
}) => {
  const theme = useBtTheme();

  // Initialize the model and the engine
  const model = useRef(new DiagramModel());
  const engine = useRef(createEngine());
  let lastClickedNodeId = "";

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
        enterSubtree(node.getName());
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

  if (render.current) {
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
    render.current = false;
  }

  return (
    <>
      <TreeMonitorMenu
        onZoomToFit={zoomToFit}
        setGoBack={setGoBack}
        subTreeName={subTreeName}
      />
      {engine.current.getModel() && (
        <StyledBTCanvas
          bgColor={theme.palette.background}
          engine={engine.current}
        />
      )}
    </>
  );
};

export default TreeMonitor;
