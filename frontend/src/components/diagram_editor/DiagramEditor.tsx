import React, { useMemo, useState, useEffect } from 'react';
import createEngine, { 
  DefaultLinkModel, 
  DefaultNodeModel,
  DiagramEngine,
  DiagramModel 
} from '@projectstorm/react-diagrams';

import {
  CanvasWidget
} from '@projectstorm/react-canvas-core';

import './DiagramEditor.css';
import { BasicNodeModel } from './nodes/basic_node/BasicNodeModel';
import { BasicNodeFactory } from './nodes/basic_node/BasicNodeFactory'; // Import custom node factory

import { TagNodeModel } from './nodes/tag_node/TagNodeModel';
import { TagNodeFactory } from './nodes/tag_node/TagNodeFactory';

import NodeHeader from './NodeHeader'; // Import HeaderMenu

const DiagramEditor = ({currentProjectname, setModelJson} : {currentProjectname : any, setModelJson : any}) => {

  // Create the engine
  const engine = useMemo(() => {
    const newEngine = createEngine();
    newEngine.getNodeFactories().registerFactory(new BasicNodeFactory());
    newEngine.getNodeFactories().registerFactory(new TagNodeFactory());
    return newEngine;
  }, []);

  // Create the model
  const model = new DiagramModel();
  const root_node = new BasicNodeModel('Tree Root', 'rgb(0,204,0)');
  root_node.setPosition(200, 200);
  root_node.addChildrenPort("Children Port");
  model.addAll(root_node);
  engine.setModel(model);

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

  // Add the nodes default ports
  const addDefaultPorts = (node:any) => {

    var nodeName = node.options.name;
    if (nodeName == "RetryUntilSuccessful") node.addInputPort("num_attempts");
    else if (nodeName == "Repeat") node.addInputPort("num_cycles");
    else if (nodeName == "Delay") node.addInputPort("delay_ms");
  }

  const nodeTypeSelector = (nodeName:any) => {

    if (["Input port value", "Output port value"].includes(nodeName)) addTagNode(nodeName);
    else addBasicNode(nodeName);
  }

  // Function to add a new node
  const addBasicNode = (nodeName:any) => {

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

    // Add ports
    if (hasInputPort) newNode.addParentPort("Parent Port");
    if (hasOutputPort) newNode.addChildrenPort("Children Port");
    addDefaultPorts(newNode);

    // Add the node to the model
    if (model) {
      model.addNode(newNode);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.serialize()));
    }
  };

  const addTagNode = (nodeName:any) => {

    const newNode = new TagNodeModel('value', 'rgb(255,153,51)'); 
    
    if (nodeName == "Input port value") newNode.addOutputPort();
    else newNode.addInputPort();

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    lastClickedNodeId = newNode.getID();
    
    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;

    // Add the node to the model
    if (model) {
      model.addNode(newNode);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.serialize()));
    }
  }

  const deleteLastClickedNode = () => {
    if (model && lastClickedNodeId) {
      const node = model.getNode(lastClickedNodeId);
      if (node) {
        node.remove();
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.serialize()));
      }
      lastClickedNodeId = "";
    }
  };

  const checkIfAction = (node:any) => {
    
    var name = node.options.name;

    // Check if the node is a user written action
    return !(["Sequence", "ReactiveSequence", "SequenceWithMemory", 
        "Fallback", "ReactiveFallback", "RetryUntilSuccessful", "Inverter", "ForceSuccess", 
        "ForceFailure", "KeepRunningUntilFailure", "Repeat", "RunOnce", "Delay",
        "Input port value", "Output port value"].includes(name))
  }

  const addInputPort = () => {

    if (model && lastClickedNodeId) {

      const genericNode = model.getNode(lastClickedNodeId);
      if (genericNode) {

        // Cast the node to BasicNodeModel
        const node = genericNode as BasicNodeModel;

        // Check restrictions
        if (checkIfAction(node)) {
          // Now you can call your custom method
          const portName = prompt("Enter the name for the new input port:");
          if (portName !== null) { // Check that the user didn't cancel
            node.addInputPort(portName);
          }
          engine.repaintCanvas();
          setModelJson(JSON.stringify(model.serialize()));
        } else {
          window.alert("Ports can only be added to action nodes")
        }
      }
    }
  };

  const addOutputPort = () => {

    if (model && lastClickedNodeId) {
      
      const genericNode = model.getNode(lastClickedNodeId);
      if (genericNode) {

        // Cast the node to BasicNodeModel
        const node = genericNode as BasicNodeModel;
        
        // Check restrictions
        if (checkIfAction(node)) {
          // Now you can call your custom method
          const portName = prompt("Enter the name for the new output port:");
          if (portName !== null) { // Check that the user didn't cancel
            node.addOutputPort(portName);
          }
          engine.repaintCanvas();
          setModelJson(JSON.stringify(model.serialize()));
        } else {
          window.alert("Ports can only be added to action nodes")
        }
      }
    }
  };

  const generateApp = () => {
    if (model) {
      const str = JSON.stringify(model.serialize());
    
      fetch("/tree_api/generate_app/", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ app_name: currentProjectname, content: str }),
      })
      .then((response) => {
        if (!response.ok) {
          return response.json().then((data) => {
            throw new Error(data.message || "An error occurred.");
          });
        }
        return response.blob();
      })
      .then((blob) => {
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.style.display = 'none';
        a.href = url;
        a.download = `${currentProjectname}.zip`;
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url);
      })
      .catch((error) => {
        console.error("Error:", error);
      });
    }
  };  
  
  return (
    <div>
      <NodeHeader 
        onNodeTypeSelected={nodeTypeSelector} 
        onDeleteNode={deleteLastClickedNode}
        onAddInputPort={addInputPort}
        onAddOutputPort={addOutputPort}
        onGenerateApp={generateApp}
        currentProjectname={currentProjectname}
      />
      <CanvasWidget className="canvas" engine={engine} />
    </div>
  );
};

export default React.memo(DiagramEditor);
