import { useRef, memo, MutableRefObject } from "react";

import createEngine, {
  DiagramModel,
  DiagramModelGenerics,
  NodeModel,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./TreeEditor.css";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";
import { ChildrenPortModel } from "./nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "./nodes/basic_node/ports/parent_port/ParentPortModel";
import { InputPortModel } from "./nodes/basic_node/ports/input_port/InputPortModel";
import { TagOutputPortModel } from "./nodes/tag_node/ports/output_port/TagOutputPortModel";

import {
  configureEngine,
  isActionNode,
  addActionFrame,
  getActionFrame,
} from "../helper/TreeEditorHelper";

const TreeEditor = memo(
  ({
    fileContent,
    setFileContent,
    setModalModel,
    setModalEngine,
    enterSubtree,
    setEditActionModalOpen,
    setEditTagModalOpen,
    setCurrentNode,
    deleteCurrentCallbackRef,
    editCurrentCallbackRef,
    homeZoomCallbackRef,
    addNodeCallbackRef,
    closeEditTagModalRef,
    closeEditActionModalRef,
    render,
  }: {
    fileContent: any;
    setFileContent: Function;
    setModalModel: Function;
    setModalEngine: Function;
    enterSubtree: Function;
    setEditActionModalOpen: Function;
    setEditTagModalOpen: Function;
    setCurrentNode: Function;
    deleteCurrentCallbackRef: MutableRefObject<(e: any) => void>;
    editCurrentCallbackRef: MutableRefObject<(e: any) => void>;
    homeZoomCallbackRef: MutableRefObject<(e: any) => void>;
    addNodeCallbackRef: MutableRefObject<(e: any) => void>;
    closeEditActionModalRef: MutableRefObject<(e: any) => void>;
    closeEditTagModalRef: MutableRefObject<(e: any) => void>;
    render: MutableRefObject<boolean>;
  }) => {
    // VARS
    // Initialize position and the last clicked node
    var lastMovedNodePosition = { x: 100, y: 100 };
    const lastClickedNodeId = useRef<string>("");

    // REFS
    // Initialize the model and the engine
    const model = useRef(new DiagramModel());
    const engine = useRef(createEngine());

    // MODAL MANAGEMENT
    const modalManager = () => {
      const node = model.current.getNode(lastClickedNodeId.current);
      lastClickedNodeId.current = "";
      model.current.clearSelection();
      if (node instanceof BasicNodeModel) {
        if (node.getIsSubtree()) {
          // Save the current subtree json
          updateJsonState();
          enterSubtree(node.getName());
        } else {
          actionEditor(node);
        }
      } else if (node instanceof TagNodeModel) {
        tagEditor(node);
      }
    };

    // HELPERS

    // Add the nodes default ports
    const addDefaultPorts = (node: BasicNodeModel | TagNodeModel) => {
      console.log("Adding default ports");

      var nodeName = node.getName();
      if (nodeName === "RetryUntilSuccessful")
        node.addInputPort("num_attempts");
      else if (nodeName === "Repeat") node.addInputPort("num_cycles");
      else if (nodeName === "Delay") node.addInputPort("delay_ms");

      var actionFrame = getActionFrame(nodeName);

      if (!(node instanceof BasicNodeModel)) {
        return;
      }

      if (actionFrame === undefined) {
        if (isActionNode(nodeName) && !node.getIsSubtree()) {
          addActionFrame(nodeName, node.getColor(), node.getPorts());
        }
        return;
      }

      node.setColor(actionFrame.getColor());

      actionFrame.getInputs().forEach((input) => {
        node.addInputPort(input);
      });

      actionFrame.getOutputs().forEach((output) => {
        node.addOutputPort(output);
      });
    };

    // Updates the json state
    const updateJsonState = () => {
      setFileContent(JSON.stringify(model.current.serialize(), null, 4));
    };

    // Deletes the last clicked node
    const deleteLastClickedNode = () => {
      if (model.current && lastClickedNodeId.current) {
        const node: NodeModel = model.current.getNode(
          lastClickedNodeId.current,
        );
        if (node) {
          node.remove();
          engine.current.repaintCanvas();
          updateJsonState();
        }
        lastClickedNodeId.current = "";
      }
    };

    // Zooms to fit the nodes
    const zoomToFit = () => {
      engine.current.zoomToFitNodes({ margin: 50 });
    };

    const onNodeEditor = () => {
      const node = model.current.getNode(lastClickedNodeId.current);
      lastClickedNodeId.current = "";
      model.current.clearSelection();
      if (node instanceof BasicNodeModel) {
        actionEditor(node);
      } else if (node instanceof TagNodeModel) {
        tagEditor(node);
      }
    };

    const actionEditor = (node: BasicNodeModel) => {
      if (isActionNode(node.getName())) {
        setCurrentNode(node);
        setEditActionModalOpen(true);
      }
    };

    const tagEditor = (node: TagNodeModel) => {
      if (isActionNode(node.getName())) {
        setCurrentNode(node);
        setEditTagModalOpen(true);
      }
    };

    const closeActionEditor = (node: BasicNodeModel) => {
      updateJsonState();
    };

    // LISTENERS

    // Position listener
    const attachPositionListener = (node: BasicNodeModel | TagNodeModel) => {
      node.registerListener({
        positionChanged: (event: any) => {
          lastMovedNodePosition = event.entity.getPosition();
          updateJsonState();
        },
      });
    };

    // Click listener
    const attachClickListener = (node: BasicNodeModel | TagNodeModel) => {
      node.registerListener({
        selectionChanged: (event: any) => {
          if (event.isSelected) {
            lastClickedNodeId.current = node.getID();
            node.selectNode();
          } else {
            node.deselectNode();
          }
        },
      });
    };

    // Link listener
    const attachLinkListener = (model: DiagramModel<DiagramModelGenerics>) => {
      model.registerListener({
        linksUpdated: (event: any) => {
          const { link, isCreated } = event;
          link.registerListener({
            targetPortChanged: (link: any) => {
              if (isCreated) {
                const { sourcePort, targetPort } = link.entity;
                if (
                  targetPort.options.alignment === "left" &&
                  Object.keys(targetPort.getLinks()).length > 1
                ) {
                  model.removeLink(link.entity);
                  sourcePort.removeLink(link.entity);
                  targetPort.removeLink(link.entity);
                } else if (
                  sourcePort instanceof ChildrenPortModel &&
                  !(targetPort instanceof ParentPortModel)
                ) {
                  model.removeLink(link.entity);
                } else if (
                  sourcePort instanceof TagOutputPortModel &&
                  !(targetPort instanceof InputPortModel)
                ) {
                  model.removeLink(link.entity);
                } else {
                  model.clearSelection();
                }
              }
              updateJsonState();
            },
          });
          updateJsonState();
        },
      });
    };

    const attachNodesListener = (model: DiagramModel<DiagramModelGenerics>) => {
      model.registerListener({
        nodesUpdated: (event: any) => {
          updateJsonState();
        },
      });
    };

    // NODE FUNCIONS

    // Function to add a new basic node
    const addBasicNode = (nodeType: string, nodeName: string) => {
      // Control parameters
      const nodeConfig = {
        default: { color: "rgb(128,0,128)", isAction: true, isSubtree: false },
        sequences: {
          color: "rgb(0,128,255)",
          isAction: false,
          isSubtree: false,
        },
        fallbacks: { color: "rgb(255,0,0)", isAction: false, isSubtree: false },
        decorators: {
          color: "rgb(255,153,51)",
          isAction: false,
          isSubtree: false,
        },
        subtrees: { color: "rgb(179,89,0)", isAction: false, isSubtree: true },
      };

      type NodeType =
        | "default"
        | "sequences"
        | "fallbacks"
        | "decorators"
        | "subtrees";

      console.log("The node type is:", nodeType.toLowerCase());
      const {
        color: nodeColor,
        isAction,
        isSubtree,
      } = nodeConfig[nodeType.toLowerCase() as NodeType] || nodeConfig.default;

      // Create node
      const newNode = new BasicNodeModel(nodeName, nodeColor, isSubtree);

      // Attach listeners
      attachPositionListener(newNode);
      attachClickListener(newNode);

      // Setup the node position
      const new_y = lastMovedNodePosition.y + 100;
      newNode.setPosition(lastMovedNodePosition.x, new_y);
      lastMovedNodePosition.y = new_y;

      // Add ports
      newNode.addParentPort("Parent Port");
      if (!isAction && !isSubtree) newNode.addChildrenPort("Children Port");
      addDefaultPorts(newNode);

      // Add the node to the model
      if (model.current) {
        model.current.addNode(newNode);
        newNode.selectNode();
        engine.current.repaintCanvas();
      }
    };

    // Function to add a new tag node
    const addTagNode = (nodeName: string) => {
      const newNode = new TagNodeModel("value", "rgb(128,128,128)");

      // Attach listeners
      attachPositionListener(newNode);
      attachClickListener(newNode);

      // Setup the node position
      var new_y = lastMovedNodePosition.y + 100;
      newNode.setPosition(lastMovedNodePosition.x, new_y);
      lastMovedNodePosition.y = new_y;

      // Add ports
      nodeName === "Input port value"
        ? newNode.addOutputPort()
        : newNode.addInputPort();

      // Add the node to the model and update the canvas
      if (model.current) {
        model.current.addNode(newNode);
        newNode.selectNode();
        // setProjectChanges(true);
        engine.current.repaintCanvas();
      }
    };

    // Select which node to add depending on the name
    const nodeTypeSelector = (e: any) => {
      const name = e.detail.name;
      const type = e.detail.type;

      console.log("Adding node:", name);
      // Unselect the previous node
      const node = model.current.getNode(lastClickedNodeId.current);
      if (node) node.setSelected(false);

      // Set the project edited flag and update the state so it can be properly saved
      updateJsonState();

      // Select depending on the name
      if (type === "Port values") addTagNode(name);
      else addBasicNode(type, name);
    };

    if (render.current) {
      // Configure the engine
      configureEngine(engine, modalManager, modalManager);

      // Deserialize and load the model
      model.current.deserializeModel(fileContent, engine.current);
      attachLinkListener(model.current);
      attachNodesListener(model.current);
      engine.current.setModel(model.current);

      // After deserialization, attach listeners to each node
      const nodes = model.current.getNodes();
      nodes.forEach((node) => {
        if (node instanceof BasicNodeModel || node instanceof TagNodeModel) {
          attachPositionListener(node);
          attachClickListener(node);
          node.setSelected(false);
          if (
            node instanceof BasicNodeModel &&
            isActionNode(node.getName()) &&
            !node.getIsSubtree()
          ) {
            addActionFrame(node.getName(), node.getColor(), node.getPorts());
          }
        }
      });

      setModalModel(model.current);
      setModalEngine(engine.current);

      // Fixes uncomplete first serialization
      setTimeout(() => {
        console.log("Rendered!");
        updateJsonState();
      }, 1);
      render.current = false;
    }

    deleteCurrentCallbackRef.current = deleteLastClickedNode;
    editCurrentCallbackRef.current = onNodeEditor;
    homeZoomCallbackRef.current = zoomToFit;
    addNodeCallbackRef.current = nodeTypeSelector;
    closeEditActionModalRef.current = closeActionEditor;
    closeEditTagModalRef.current = closeActionEditor;

    return (
      <>
        {engine.current && (
          <CanvasWidget className="bt-canvas" engine={engine.current} />
        )}
      </>
    );
  },
);

export default TreeEditor;
