import React from 'react';
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
  const addNode = (nodeType : any) => {

    const newNode = new SpecialNodeModel(nodeType, 'rgb(255,153,51)');
    newNode.setPosition(Math.random() * 400, Math.random() * 400);
    newNode.addChildrenPort("Children Port");
    newNode.addParentPort("Parent Port");
    model.addNode(newNode);
    engine.repaintCanvas();
  };

  return (
    <div>
      <NodeHeader onNodeTypeSelected={addNode} />
      <CanvasWidget className="canvas" engine={engine} />
    </div>
  );
};

export default DiagramEditor;
