import React, { useMemo, useState } from 'react';
import createEngine, { 
  DefaultLinkModel, 
  DefaultNodeModel,
  DiagramModel 
} from '@projectstorm/react-diagrams';

import {
  CanvasWidget
} from '@projectstorm/react-canvas-core';

import { BasicNodeFactory } from './nodes/basic_node/BasicNodeFactory'; // Import custom node factory

import './DiagramEditor.css';
import { BasicNodeModel } from './nodes/basic_node/BasicNodeModel';
import NodeHeader from './NodeHeader'; // Import HeaderMenu

const DiagramEditor = () => {

  // Initial node position
  let lastMovedNodePosition = { x: 200, y: 200 };

  // Initialize state for last moved node ID
  let lastClickedNodeId = "";

  const attachPositionListener = (node:any) => {
    node.registerListener({
      positionChanged: (event:any) => {
        lastMovedNodePosition = event.entity.getPosition();
      },
    });
  };

  const attachClickListener = (node:any) => {
    node.registerListener({
      selectionChanged: (event:any) => {
        if (event.isSelected) {
          lastClickedNodeId = node.getID();
        }
      },
    });
  };

  // create an instance of the engine with all the defaults
  const engine = createEngine();

  // Register the custom node factory
  engine.getNodeFactories().registerFactory(new BasicNodeFactory());

  // Root node
  const root_node = new BasicNodeModel('Tree Root', 'rgb(0,204,0)')
  root_node.setPosition(200, 200);
  root_node.addChildrenPort("Children Port")

  const model = new DiagramModel();
  model.addAll(root_node);

  engine.setModel(model);

  // Function to add a new node
  const addNode = (nodeName:any) => {

    // Control parameters
    let nodeColor = 'rgb(255,153,51)'; // Default color
    let hasInputPort = true;
    let hasOutputPort = true;

    // Sequences
    if (["Sequence", "ReactiveSequence", "SequenceWithMemory"].includes(nodeName)) {
      nodeColor = 'rgb(0,128,255)';
    }
    // Fallbacks
    else if (["Fallback", "ReactiveFallback"].includes(nodeName)) {
      nodeColor = 'rgb(255,0,0)';
    }
    // Decorators
    else if (["RetryUntilSuccessful", "Inverter", "ForceSuccess", "ForceFailure", "KeepRunningUntilFailure", "Repeat", "RunOnce", "Delay"].includes(nodeName)) {
      nodeColor = 'rgb(255,153,51)';
    }
    // Actions
    else {
      nodeColor = 'rgb(128,0,128)';
      hasOutputPort = false;
    }

    // Create node
    const newNode = new BasicNodeModel(nodeName, nodeColor);

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    lastClickedNodeId = newNode.getID();

    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;
    if (hasInputPort) newNode.addParentPort("Parent Port");
    if (hasOutputPort) newNode.addChildrenPort("Children Port");

    model.addNode(newNode);
    engine.repaintCanvas();
  };

  const deleteLastClickedNode = () => {
    if (lastClickedNodeId) {
      const node = model.getNode(lastClickedNodeId);
      if (node) {
        node.remove();
        engine.repaintCanvas();
      }
      lastClickedNodeId = "";
    }
  };

  const addInputPort = () => {
    if (lastClickedNodeId) {
      const genericNode = model.getNode(lastClickedNodeId);
      if (genericNode) {
        // Cast the node to BasicNodeModel
        const node = genericNode as BasicNodeModel;
        
        // Now you can call your custom method
        const portName = prompt("Enter the name for the new input port:");
        if (portName !== null) { // Check that the user didn't cancel
          node.addInputPort(portName);
        }
        engine.repaintCanvas();
      }
    }
  };

  const addOutputPort = () => {
    if (lastClickedNodeId) {
      const genericNode = model.getNode(lastClickedNodeId);
      if (genericNode) {
        // Cast the node to BasicNodeModel
        const node = genericNode as BasicNodeModel;
        
        // Now you can call your custom method
        const portName = prompt("Enter the name for the new output port:");
        if (portName !== null) { // Check that the user didn't cancel
          node.addOutputPort(portName);
        }
        engine.repaintCanvas();
      }
    }
  };

  return (
    <div>
      <NodeHeader 
        onNodeTypeSelected={addNode} 
        onDeleteNode={deleteLastClickedNode}
        onAddInputPort={addInputPort}
        onAddOutputPort={addOutputPort}
      />
      <CanvasWidget className="canvas" engine={engine} />
    </div>
  );
};

export default React.memo(DiagramEditor);
