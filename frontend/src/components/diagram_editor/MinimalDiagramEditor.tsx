import React from "react";
import { useRef, useMemo } from "react";

import createEngine, {
  DefaultLinkModel,
  DefaultNodeModel,
  DiagramModel,
} from "@projectstorm/react-diagrams";
import { CanvasWidget } from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { BasicNodeFactory } from "./nodes/basic_node/BasicNodeFactory";
import { TagNodeFactory } from "./nodes/tag_node/TagNodeFactory";
import { SimplePortFactory } from "./nodes/SimplePortFactory";
import { ChildrenPortModel } from "./nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "./nodes/basic_node/ports/parent_port/ParentPortModel";
import { OutputPortModel } from "./nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "./nodes/basic_node/ports/input_port/InputPortModel";
import { TagOutputPortModel } from "./nodes/tag_node/ports/output_port/TagOutputPortModel";
import { TagInputPortModel } from "./nodes/tag_node/ports/input_port/TagInputPortModel";

const testFunction = () => {
  console.log("Hello!");
};

const MinimalDiagramEditor = ({ modelJson }: { modelJson: any }) => {
  // Initialize the model and the engine
  const model = useRef(new DiagramModel());

  // Initialize the engine with its factories
  const engine = useRef(createEngine());
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

  model.current.deserializeModel(modelJson, engine.current);
  engine.current.setModel(model.current);
  console.log(model.current);

  return <CanvasWidget className="canvas" engine={engine.current} />;
};

export default MinimalDiagramEditor;
