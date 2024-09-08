import React, { useEffect, useState } from "react";
import { useRef, memo } from "react";

import createEngine, {
  DefaultLinkModel,
  DefaultNodeModel,
  DiagramModel,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
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

import NodeMenu from "./NodeMenu";

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

// Add the nodes default ports
const addDefaultPorts = (node: any) => {
  console.log("Adding default ports");

  var nodeName = node.getName();
  if (nodeName === "RetryUntilSuccessful") node.addInputPort("num_attempts");
  else if (nodeName === "Repeat") node.addInputPort("num_cycles");
  else if (nodeName === "Delay") node.addInputPort("delay_ms");
};

const MinimalDiagramEditor = memo(
  ({
    modelJson,
    setResultJson,
    projectName,
    setDiagramEdited,
  }: {
    modelJson: any;
    setResultJson: Function;
    projectName: string;
    setDiagramEdited: Function;
  }) => {
    // VARS

    // Initialize position and the last clicked node
    var lastMovedNodePosition = { x: 100, y: 100 };
    var lastClickedNodeId = "";

    // REFS

    // Initialize the model and the engine
    const model = useRef(new DiagramModel());
    const engine = useRef(createEngine());

    // HELPERS
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

    // Function to add a new basic node
    const addBasicNode = (nodeName: string) => {
      // Control parameters
      const nodeConfig = {
        default: {
          color: "rgb(128,0,128)",
          isAction: true,
        },
        sequences: {
          color: "rgb(0,128,255)",
          isAction: false,
        },
        fallbacks: {
          color: "rgb(255,0,0)",
          isAction: false,
        },
        decorators: {
          color: "rgb(255,153,51)",
          isAction: false,
        },
      };

      const sequenceNodes = [
        "Sequence",
        "ReactiveSequence",
        "SequenceWithMemory",
      ];
      const fallbackNodes = ["Fallback", "ReactiveFallback"];
      const decoratorNodes = [
        "RetryUntilSuccessful",
        "Inverter",
        "ForceSuccess",
        "ForceFailure",
        "KeepRunningUntilFailure",
        "Repeat",
        "RunOnce",
        "Delay",
      ];

      let nodeColor, hasInputPort, hasOutputPort, isAction;

      if (sequenceNodes.includes(nodeName)) {
        ({ color: nodeColor, isAction } = nodeConfig.sequences);
      } else if (fallbackNodes.includes(nodeName)) {
        ({ color: nodeColor, isAction } = nodeConfig.fallbacks);
      } else if (decoratorNodes.includes(nodeName)) {
        ({ color: nodeColor, isAction } = nodeConfig.decorators);
      } else {
        ({ color: nodeColor, isAction } = nodeConfig.default);
      }

      // Create node
      const newNode = new BasicNodeModel(nodeName, nodeColor);

      // Attach listeners
      attachPositionListener(newNode);
      attachClickListener(newNode);

      // Setup the node position
      var new_y = lastMovedNodePosition.y + 100;
      newNode.setPosition(lastMovedNodePosition.x, new_y);
      lastMovedNodePosition.y = new_y;

      // Add ports
      newNode.addParentPort("Parent Port");
      if (!isAction) newNode.addChildrenPort("Children Port");
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
    const nodeTypeSelector = (nodeName: any) => {
      // Unselect the previous node
      const node = model.current.getNode(lastClickedNodeId);
      if (node) node.setSelected(false);

      // Set the project edited flag and update the state so it can be properly saved
      setDiagramEdited(true);
      updateJsonState();

      // Select depending on the name
      if (["Input port value", "Output port value"].includes(nodeName))
        addTagNode(nodeName);
      else addBasicNode(nodeName);
    };

    // There is no need to use an effect as the editor will re render when the model json changes
    // Configure the engine
    configureEngine(engine);

    // Deserialize and load the model
    model.current.deserializeModel(modelJson, engine.current);
    setResultJson(modelJson);
    attachLinkListener(model.current);
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
        <NodeMenu
          projectName={projectName}
          onAddNode={nodeTypeSelector}
          onDeleteNode={deleteLastClickedNode}
          onZoomToFit={zoomToFit}
          onEditAction={actionEditor}
        />
        <CanvasWidget className="canvas" engine={engine.current} />
      </div>
    );
  }
);

export default MinimalDiagramEditor;
