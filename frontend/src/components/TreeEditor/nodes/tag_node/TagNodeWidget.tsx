import React, { JSX } from "react";
import { InputPortWidget } from "./ports/input_port/TagInputPortWidget";
import { OutputPortWidget } from "./ports/output_port/TagOutputPortWidget";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import { StyledNodeSection, StyledTagContainer } from "Styles/TreeEditor/BTNode.styles";
import { useBtTheme } from "Contexts/BtThemeContext";

// The node widget controls the visualization of the custom node
export const TagNodeWidget = ({
  engine,
  node,
}: {
  engine: DiagramEngine;
  node: any;
}) => {
  const theme = useBtTheme();

  // Ports list
  const inputPorts: JSX.Element[] = [];
  const outputPorts: JSX.Element[] = [];

  // Initial class
  var type: "input" | "output" = "input";

  // Get all the ports from the node and classify them
  Object.keys(node.getPorts()).forEach((portName) => {
    const port = node.getPort(portName);
    if (!port) return;

    if (port.options.type === "tag input") {
      inputPorts.push(
        <InputPortWidget key={portName} engine={engine} port={port} />
      );
    } else if (port.options.type === "tag output") {
      outputPorts.push(
        <OutputPortWidget key={portName} engine={engine} port={port} />
      );
    }
  });

  // Adjust class name depending on the quantity of ports
  if (inputPorts.length === 0 && outputPorts.length > 0) {
    type = "output";
  }

  // Return the node to render
  return (
    <StyledTagContainer
      borderColor={theme.btEditor.border}
      roundness={theme.btEditor.roundness}
      color={node.getColor()}
      selected={node.isSelected()}
      type={type}
      shadowColor={theme.btEditor.shadow}
    >
      <StyledNodeSection>
        {inputPorts}
        {node.getName()}
        {outputPorts}
      </StyledNodeSection>
    </StyledTagContainer>
  );
};
