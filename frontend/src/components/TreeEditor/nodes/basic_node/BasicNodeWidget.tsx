import React from "react";
import { ChildrenPortWidget } from "./ports/children_port/ChildrenPortWidget";
import { ParentPortWidget } from "./ports/parent_port/ParentPortWidget";
import { InputPortWidget } from "./ports/input_port/InputPortWidget";
import { OutputPortWidget } from "./ports/output_port/OutputPortWidget";

import { strRGBToLuminance } from "../../../helper/colorHelper";

import "./BasicNode.css";
import { DiagramEngine } from "@projectstorm/react-diagrams";

// The node widget controls the visualization of the custom node
export const BasicNodeWidget = ({
  engine,
  node,
}: {
  engine: DiagramEngine;
  node: any;
}) => {
  // Choose the font color
  let showLightText = strRGBToLuminance(node.getColor()) <= 0.5;

  // Node style
  let nodeStyle: React.CSSProperties = {
    background: node.getColor() || "var(--bt-action-default-color)",
    color: showLightText ? "var(--bt-light-text)" : "var(--bt-dark-text)",
    ...(node.isSelected() && {
      boxShadow: "0 0 12px var(--bt-selected-shadow-color)", // Add a shadow to highlight the selection
    }),
  };

  switch (node.getExecStatus()) {
    case "RUNNING":
      nodeStyle["background"] = "var(--bt-status-running)";
      break;
    case "SUCCESS":
      nodeStyle["background"] = "var(--bt-status-success)";
      break;
    case "FAILURE":
      nodeStyle["background"] = "var(--bt-status-failure)";
      break;
    case "INVALID":
      nodeStyle["background"] = "var(--bt-status-invalid)";
      break;
    default:
      break;
  }

  // Ports list
  const parentPorts: JSX.Element[] = [];
  const childrenPorts: JSX.Element[] = [];
  const inputPorts: JSX.Element[] = [];
  const outputPorts: JSX.Element[] = [];

  // Get all the ports from the node and classify them
  Object.keys(node.getPorts()).forEach((portName) => {
    const port = node.getPort(portName);
    if (!port) return;

    if (port.options.type === "parent") {
      parentPorts.push(
        <ParentPortWidget key={portName} engine={engine} port={port} />,
      );
    } else if (port.options.type === "children") {
      childrenPorts.push(
        <ChildrenPortWidget key={portName} engine={engine} port={port} />,
      );
    } else if (port.options.type === "input") {
      inputPorts.push(
        <InputPortWidget key={portName} engine={engine} port={port} />,
      );
    } else if (port.options.type === "output") {
      outputPorts.push(
        <OutputPortWidget key={portName} engine={engine} port={port} />,
      );
    }
  });

  // Return the node to render
  return (
    <div className="bt-basic-node" style={nodeStyle}>
      <div className="bt-basic-layer">
        <div className="bt-basic-parent-port">
          {parentPorts.length > 0 ? (
            parentPorts
          ) : (
            <div className="bt-basic-placeholder"></div>
          )}
        </div>
        <div className="bt-basic-title">{node.getName()}</div>
        <div className="bt-basic-children-port">
          {childrenPorts.length > 0 ? (
            childrenPorts
          ) : (
            <div className="bt-basic-placeholder"></div>
          )}
        </div>
      </div>
      <div className="bt-basic-layer">
        <div className="bt-basic-left-ports">{inputPorts}</div>
        <div className="bt-basic-right-ports">{outputPorts}</div>
      </div>
    </div>
  );
};
