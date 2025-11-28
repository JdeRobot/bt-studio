import React, { ReactElement } from "react";
import { InputPortWidget } from "./ports/input_port/TagInputPortWidget";
import { OutputPortWidget } from "./ports/output_port/TagOutputPortWidget";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import {
  StyledNodeSection,
  StyledTagContainer,
} from "BtStyles/TreeEditor/BTNode.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";

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
  const inputPorts: ReactElement[] = [];
  const outputPorts: ReactElement[] = [];

  // Initial class
  let type: "input" | "output" = "input";

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
  if (inputPorts.length === 0 && outputPorts.length > 0) {
    type = "output";
  }

  // Return the node to render
  return (
    <StyledTagContainer
      borderColor={theme.btEditor.border}
      roundness={theme.btEditor.roundness}
      bg={node.isFromBlackboard() ? theme.btEditor.blackboard : theme.btEditor.tag}
      lightText={theme.btEditor.lightText}
      darkText={theme.btEditor.darkText}
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
