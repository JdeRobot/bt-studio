import React from "react";
import { InputPortWidget } from "./ports/input_port/TagInputPortWidget";
import { OutputPortWidget } from "./ports/output_port/TagOutputPortWidget";

import "./TagNode.css";
import { DiagramEngine } from "@projectstorm/react-diagrams";

// The node widget controls the visualization of the custom node
export const TagNodeWidget = ({
  engine,
  node,
}: {
  engine: DiagramEngine;
  node: any;
}) => {
  // Tag style
  let tagStyle: React.CSSProperties = {
    background: node.getColor(),
    ...(node.isSelected() && {
      boxShadow: "0 0 12px var(--bt-selected-shadow-color)", // Add a shadow to highlight the selection
    }),
  };

  // Ports list
  const inputPorts: JSX.Element[] = [];
  const outputPorts: JSX.Element[] = [];

  // Initial class
  let nodeClass = "tag-node";

  // Get all the ports from the node and classify them
  Object.keys(node.getPorts()).forEach((portName) => {
    const port = node.getPort(portName);
    if (!port) return;

    if (port.options.type === "tag input") {
      inputPorts.push(
        <InputPortWidget key={portName} engine={engine} port={port} />,
      );
    } else if (port.options.type === "tag output") {
      outputPorts.push(
        <OutputPortWidget key={portName} engine={engine} port={port} />,
      );
    }
  });

  // Adjust class name depending on the quantity of ports
  if (inputPorts.length > 0 && outputPorts.length === 0) {
    nodeClass = "bt-tag-node bt-tag-input";
  } else if (inputPorts.length === 0 && outputPorts.length > 0) {
    nodeClass = "bt-tag-node bt-tag-output";
  }

  // Return the node to render
  return (
    <div className={nodeClass} style={tagStyle}>
      <div className="bt-tag-layer">
        {inputPorts}
        {node.getName()}
        {outputPorts}
      </div>
    </div>
  );
};
