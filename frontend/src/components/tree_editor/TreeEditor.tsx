import React, { useEffect, useState } from "react";
import { useRef, memo } from "react";

import createEngine, {
  DefaultLinkModel,
  DefaultNodeModel,
  DiagramModel,
  ZoomCanvasAction,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./TreeEditor.css";
import { BasicNodeFactory } from "./nodes/basic_node/BasicNodeFactory";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { TagNodeFactory } from "./nodes/tag_node/TagNodeFactory";
import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";
import { SimplePortFactory } from "./nodes/SimplePortFactory";
import { ChildrenPortModel } from "./nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "./nodes/basic_node/ports/parent_port/ParentPortModel";
import { OutputPortModel } from "./nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "./nodes/basic_node/ports/input_port/InputPortModel";
import { TagOutputPortModel } from "./nodes/tag_node/ports/output_port/TagOutputPortModel";
import { TagInputPortModel } from "./nodes/tag_node/ports/input_port/TagInputPortModel";

import SubtreeModal from "./modals/SubTreeModal";
import NodeMenu from "./NodeMenu";

const TreeEditor = memo(
  ({
    modelJson,
    setResultJson,
    projectName,
    setDiagramEdited,
    hasSubtrees,
  }: {
    modelJson: any;
    setResultJson: Function;
    projectName: string;
    setDiagramEdited: Function;
    hasSubtrees: boolean;
  }) => {
    const [subtreeModalOpen, setSubTreeModalOpen] = useState(false);
    const [subTreeName, setSubTreeName] = useState("");

    const onSubTreeModalClose = () => {
      setSubTreeModalOpen(false);
    };

    return (
      <div>
        {hasSubtrees && subtreeModalOpen && (
          <SubtreeModal
            isOpen={subtreeModalOpen}
            onClose={onSubTreeModalClose}
            projectName={projectName}
            subtreeName={subTreeName}
            setDiagramEdited={setDiagramEdited}
          />
        )}
        <DiagramEditor
          modelJson={modelJson}
          setResultJson={setResultJson}
          projectName={projectName}
          setDiagramEdited={setDiagramEdited}
          hasSubtrees={hasSubtrees}
          setSubTreeModalOpen={setSubTreeModalOpen}
          setSubTreeName={setSubTreeName}
        />
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
    hasSubtrees,
    setSubTreeModalOpen,
    setSubTreeName,
  }: {
    modelJson: any;
    setResultJson: Function;
    projectName: string;
    setDiagramEdited: Function;
    hasSubtrees: boolean;
    setSubTreeModalOpen: Function;
    setSubTreeName: Function;
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
      if (node instanceof BasicNodeModel && node.getIsSubtree()) {
        setSubTreeName(node.getName());
        setSubTreeModalOpen(true);
      }
    };

    // HELPERS

    // Configures an engine with all the factories
    const configureEngine = (engine: any) => {
      console.log("Configuring engine!");
      // Register factories
      engine.current
        .getNodeFactories()
        .registerFactory(new BasicNodeFactory(modalManager));
      engine.current
        .getNodeFactories()
        .registerFactory(new TagNodeFactory(modalManager));
      engine.current
        .getPortFactories()
        .registerFactory(
          new SimplePortFactory(
            "children",
            (config) => new ChildrenPortModel(),
          ),
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
          new SimplePortFactory(
            "tag output",
            (config) => new TagOutputPortModel(),
          ),
        );
      engine.current
        .getPortFactories()
        .registerFactory(
          new SimplePortFactory(
            "tag input",
            (config) => new TagInputPortModel(),
          ),
        );

      // Disable loose links
      const state: any = engine.current.getStateMachine().getCurrentState();
      state.dragNewLink.config.allowLooseLinks = false;

      engine.current
        .getActionEventBus()
        .registerAction(new ZoomCanvasAction({ inverseZoom: true }));
    };

    // Add the nodes default ports
    const addDefaultPorts = (node: any) => {
      console.log("Adding default ports");

      var nodeName = node.getName();
      if (nodeName === "RetryUntilSuccessful")
        node.addInputPort("num_attempts");
      else if (nodeName === "Repeat") node.addInputPort("num_cycles");
      else if (nodeName === "Delay") node.addInputPort("delay_ms");
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

    const actionEditor = () => {
      console.log("Editing the action!");
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
    configureEngine(engine);

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
          onEditAction={actionEditor}
          hasSubtrees={hasSubtrees}
        />
        {engine.current && (
          <CanvasWidget className="canvas" engine={engine.current} />
        )}
      </div>
    );
  },
);

export default TreeEditor;
