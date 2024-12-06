import React, { useEffect, useState } from "react";
import { useRef, memo } from "react";

import createEngine, {
  DefaultLinkModel,
  DefaultNodeModel,
  DiagramEngine,
  DiagramModel,
  NodeModel,
  ZoomCanvasAction,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./TreeEditor.css";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";
import { ChildrenPortModel } from "./nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "./nodes/basic_node/ports/parent_port/ParentPortModel";
import { OutputPortModel } from "./nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "./nodes/basic_node/ports/input_port/InputPortModel";
import { TagOutputPortModel } from "./nodes/tag_node/ports/output_port/TagOutputPortModel";

import {
  configureEngine,
  isActionNode,
  TreeViewType,
} from "../helper/TreeEditorHelper";

import NodeMenu from "./NodeMenu";
import EditActionModal from "./modals/EditActionModal";
import EditTagModal from "./modals/EditTagModal";

import DiagramVisualizer from "./DiagramVisualizer";
import CommsManager from "../../api_helper/CommsManager";
import { OptionsContext } from "../options/Options";

import "./TreeEditor.css";

const TreeEditor = memo(
  ({
    modelJson,
    setResultJson,
    projectName,
    setDiagramEdited,
    hasSubtrees,
    treeStructure,
    view,
    changeView,
    setSubTreeName,
    subTreeName,
    setGoBack,
    subTreeStructure,
  }: {
    modelJson: any;
    setResultJson: Function;
    projectName: string;
    setDiagramEdited: React.Dispatch<React.SetStateAction<boolean>>;
    hasSubtrees: boolean;
    treeStructure: any;
    view: TreeViewType;
    changeView: Function;
    setSubTreeName: Function;
    subTreeName: string;
    setGoBack: Function;
    subTreeStructure: number[];
  }) => {
    const settings = React.useContext(OptionsContext);

    const [editActionModalOpen, setEditActionModalOpen] = useState(false);
    const [currentNode, setCurrentNode] = useState<
      BasicNodeModel | TagNodeModel | undefined
    >(undefined);
    const [editTagModalOpen, setEditTagModalOpen] = useState(false);
    const [btOrder, setBtOrder] = useState(settings.btOrder.default_value);

    // Model and Engine for models use
    const [modalModel, setModalModel] = useState<DiagramModel | undefined>(
      undefined,
    );
    const [modalEngine, setModalEngine] = useState<DiagramEngine | undefined>(
      undefined,
    );

    useEffect(() => {
      setBtOrder(settings.btOrder.value);
    }, [settings.btOrder.value]);

    const updateJsonState = () => {
      if (modalModel) {
        setResultJson(modalModel.serialize());
      }
    };

    const onEditActionModalClose = () => {
      setEditActionModalOpen(false);
      setCurrentNode(undefined);
    };

    const onEditTagModalClose = () => {
      setEditTagModalOpen(false);
      setCurrentNode(undefined);
    };

    return (
      <div>
        {currentNode && modalModel && modalEngine && (
          <>
            {currentNode instanceof BasicNodeModel && (
              <EditActionModal
                isOpen={editActionModalOpen}
                onClose={onEditActionModalClose}
                currentActionNode={currentNode}
                model={modalModel}
                engine={modalEngine}
                updateJsonState={updateJsonState}
                setDiagramEdited={setDiagramEdited}
              />
            )}
            {currentNode instanceof TagNodeModel && (
              <EditTagModal
                isOpen={editTagModalOpen}
                onClose={onEditTagModalClose}
                currentActionNode={currentNode}
                model={modalModel}
                engine={modalEngine}
                updateJsonState={updateJsonState}
                setDiagramEdited={setDiagramEdited}
              />
            )}
          </>
        )}
        {view === TreeViewType.Visualizer ? (
          <DiagramVisualizer
            modelJson={modelJson}
            setResultJson={setResultJson}
            manager={CommsManager.getInstance()}
            treeStructure={treeStructure}
            view={view}
            changeView={changeView}
            setGoBack={setGoBack}
            subTreeName={subTreeName}
            subTreeStructure={subTreeStructure}
            setSubTreeName={setSubTreeName}
          />
        ) : (
          <DiagramEditor
            modelJson={modelJson}
            setResultJson={setResultJson}
            projectName={projectName}
            setDiagramEdited={setDiagramEdited}
            view={view}
            changeView={changeView}
            hasSubtrees={hasSubtrees}
            setModalModel={setModalModel}
            setModalEngine={setModalEngine}
            setSubTreeName={setSubTreeName}
            setEditActionModalOpen={setEditActionModalOpen}
            setEditTagModalOpen={setEditTagModalOpen}
            setCurrentNode={setCurrentNode}
            setGoBack={setGoBack}
            subTreeName={subTreeName}
          />
        )}
        <button className="bt-order-indicator" title={"BT Order: " + btOrder}>
          <svg
            className="w-6 h-6 text-gray-800 dark:text-white"
            aria-hidden="true"
            xmlns="http://www.w3.org/2000/svg"
            width="24"
            height="24"
            fill="none"
            viewBox="0 0 24 24"
          >
            {btOrder === "bottom-to-top" ? (
              <path
                stroke="var(--background)"
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth="2"
                d="M12 6v13m0-13 4 4m-4-4-4 4"
              />
            ) : (
              <path
                stroke="var(--background)"
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth="2"
                d="M12 19V5m0 14-4-4m4 4 4-4"
              />
            )}
          </svg>
        </button>
      </div>
    );
  },
);

const DiagramEditor = memo(
  ({
    modelJson,
    setResultJson,
    projectName,
    setDiagramEdited,
    view,
    changeView,
    hasSubtrees,
    setModalModel,
    setModalEngine,
    setSubTreeName,
    setEditActionModalOpen,
    setEditTagModalOpen,
    setCurrentNode,
    setGoBack,
    subTreeName,
  }: {
    modelJson: any;
    setResultJson: Function;
    projectName: string;
    setDiagramEdited: React.Dispatch<React.SetStateAction<boolean>>;
    view: TreeViewType;
    changeView: Function;
    hasSubtrees: boolean;
    setModalModel: Function;
    setModalEngine: Function;
    setSubTreeName: Function;
    setEditActionModalOpen: Function;
    setEditTagModalOpen: Function;
    setCurrentNode: Function;
    setGoBack: Function;
    subTreeName: string;
  }) => {
    // VARS

    // Initialize position and the last clicked node
    var lastMovedNodePosition = { x: 100, y: 100 };
    var lastClickedNodeId = "";

    // REFS

    // Initialize the model and the engine
    const model = useRef(new DiagramModel());
    const engine = useRef(createEngine());

    // MODAL MANAGEMENT
    const modalManager = () => {
      const node = model.current.getNode(lastClickedNodeId);
      lastClickedNodeId = "";
      model.current.clearSelection();
      if (node instanceof BasicNodeModel) {
        if (node.getIsSubtree()) {
          // Save the current subtree json
          updateJsonState();

          setSubTreeName(node.getName());
        } else {
          actionEditor(node);
        }
      } else if (node instanceof TagNodeModel) {
        tagEditor(node);
      }
    };

    const changeViewExpanded = (view: TreeViewType) => {
      updateJsonState();
      changeView(view);
    }

    // HELPERS

    // Add the nodes default ports
    const addDefaultPorts = (node: any) => {
      console.log("Adding default ports");

      var nodeName = node.getName();
      if (nodeName === "RetryUntilSuccessful")
        node.addInputPort("num_attempts");
      else if (nodeName === "Repeat") node.addInputPort("num_cycles");
      else if (nodeName === "Delay") node.addInputPort("delay_ms");

      model.current.getNodes().forEach((oldNode: NodeModel) => {
        //TODO: for the tags, this will never be called. Maybe have a common type
        if (oldNode instanceof BasicNodeModel) {
          var convNode = oldNode as BasicNodeModel;
          if (convNode.getName() === node.getName() && node !== convNode) {
            node.setColor(convNode.getColor());
            Object.values(convNode.getPorts()).forEach((element) => {
              if (element instanceof InputPortModel) {
                node.addInputPort(element.getName());
              } else if (element instanceof OutputPortModel) {
                node.addOutputPort(element.getName());
              }
            });
          }
        }
      });
    };

    // Updates the json state
    const updateJsonState = () => {
      setResultJson(model.current.serialize());
    };

    // Deletes the last clicked node
    const deleteLastClickedNode = () => {
      if (model.current && lastClickedNodeId) {
        const node: any = model.current.getNode(lastClickedNodeId);
        if (node) {
          node.remove();
          setDiagramEdited(true);
          engine.current.repaintCanvas();
          updateJsonState();
        }
        lastClickedNodeId = "";
      }
    };

    // Zooms to fit the nodes
    const zoomToFit = () => {
      engine.current.zoomToFitNodes({ margin: 50 });
    };

    const onNodeEditor = () => {
      const node = model.current.getNode(lastClickedNodeId);
      lastClickedNodeId = "";
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

    // LISTENERS

    // Position listener
    const attachPositionListener = (node: any) => {
      node.registerListener({
        positionChanged: (event: any) => {
          lastMovedNodePosition = event.entity.getPosition();
          setDiagramEdited(true);
          updateJsonState();
        },
      });
    };

    // Click listener
    const attachClickListener = (node: any) => {
      node.registerListener({
        selectionChanged: (event: any) => {
          if (event.isSelected) {
            lastClickedNodeId = node.getID();
            node.selectNode();
          } else {
            node.deselectNode();
          }
        },
      });
    };

    // Link listener
    const attachLinkListener = (model: any) => {
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
            },
          });
          updateJsonState();
          setDiagramEdited(true);
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
    const nodeTypeSelector = (nodeType: string, nodeName: string) => {
      console.log("Adding node:", nodeName);
      // Unselect the previous node
      const node = model.current.getNode(lastClickedNodeId);
      if (node) node.setSelected(false);

      // Set the project edited flag and update the state so it can be properly saved
      setDiagramEdited(true);
      updateJsonState();

      // Select depending on the name
      if (nodeType === "Port values") addTagNode(nodeName);
      else addBasicNode(nodeType, nodeName);
    };

    // Configure the engine
    configureEngine(engine, modalManager, modalManager);

    // Deserialize and load the model
    model.current.deserializeModel(modelJson, engine.current);
    attachLinkListener(model.current);
    engine.current.setModel(model.current);

    // After deserialization, attach listeners to each node
    const nodes = model.current.getNodes();
    nodes.forEach((node) => {
      attachPositionListener(node);
      attachClickListener(node);
      node.setSelected(false);
    });

    setModalModel(model.current);
    setModalEngine(engine.current);

    // Fixes uncomplete first serialization
    setTimeout(() => {
      console.log("Rendered!");
      updateJsonState();
    }, 1);

    return (
      <div>
        <NodeMenu
          projectName={projectName}
          onAddNode={nodeTypeSelector}
          onDeleteNode={deleteLastClickedNode}
          onZoomToFit={zoomToFit}
          onEditAction={onNodeEditor}
          hasSubtrees={hasSubtrees}
          view={view}
          changeView={changeViewExpanded}
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

export default TreeEditor;
