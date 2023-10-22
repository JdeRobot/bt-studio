import React, { useMemo, useState } from 'react';
import createEngine, { 
  DefaultLinkModel, 
  DefaultNodeModel,
  DiagramModel 
} from '@projectstorm/react-diagrams';

import {
  CanvasWidget
} from '@projectstorm/react-canvas-core';

import { SpecialNodeFactory } from './nodes/SpecialNodeFactory'; // Import custom node factory

import './DiagramEditor.css';
import { SpecialNodeModel } from './nodes/SpecialNodeModel';
import NodeHeader from './NodeHeader'; // Import HeaderMenu

const DiagramEditor = () => {

  // Initial node position
  let lastMovedNodePosition = { x: 200, y: 200 };

  // Initialize state for last moved node ID
  let lastMovedNodeId = "";

  const attachPositionListener = (node:any) => {
    node.registerListener({
      positionChanged: (event:any) => {
        lastMovedNodePosition = event.entity.getPosition();
        lastMovedNodeId = node.getID();
      },
    });
  };

  // create an instance of the engine with all the defaults
  const engine = createEngine();

  // Register the custom node factory
  engine.getNodeFactories().registerFactory(new SpecialNodeFactory());

  // Root node
  const root_node = new SpecialNodeModel('Tree Root', 'rgb(0,204,0)')
  root_node.setPosition(200, 200);
  root_node.addChildrenPort("Children Port")

  const model = new DiagramModel();
  model.addAll(root_node);

  engine.setModel(model);

  // Function to add a new node
  const addNode = (nodeName:any) => {

    console.log("adding node");

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
    const newNode = new SpecialNodeModel(nodeName, nodeColor);

    // Attach listener to this node
    attachPositionListener(newNode);

    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;
    if (hasInputPort) newNode.addParentPort("Parent Port");
    if (hasOutputPort) newNode.addChildrenPort("Children Port");

    model.addNode(newNode);
    engine.repaintCanvas();
  };

  const deleteLastMovedNode = () => {
    if (lastMovedNodeId) {
      const node = model.getNode(lastMovedNodeId);
      if (node) {
        node.remove();
        engine.repaintCanvas();
      }
      lastMovedNodeId = "";
    }
  };

  return (
    <div>
      <NodeHeader 
        onNodeTypeSelected={addNode} 
        onDeleteNode={deleteLastMovedNode}  // Pass the delete function
      />
      <CanvasWidget className="canvas" engine={engine} />
    </div>
  );
};

export default React.memo(DiagramEditor);
