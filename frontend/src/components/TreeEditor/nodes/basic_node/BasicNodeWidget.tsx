import React, { ReactElement } from "react";
import { ChildrenPortWidget } from "./ports/children_port/ChildrenPortWidget";
import { ParentPortWidget } from "./ports/parent_port/ParentPortWidget";
import { InputPortWidget } from "./ports/input_port/InputPortWidget";
import { OutputPortWidget } from "./ports/output_port/OutputPortWidget";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import {
  StyledNodeContainer,
  StyledNodeDiagramPorts,
  StyledNodePort,
  StyledNodeSection,
  StyledNodeTagPorts,
  StyledNodeTitle,
} from "Styles/TreeEditor/BTNode.styles";
import { useBtTheme } from "Contexts/BtThemeContext";

// The node widget controls the visualization of the custom node
export const BasicNodeWidget = ({
  engine,
  node,
}: {
  engine: DiagramEngine;
  node: any;
}) => {
  const theme = useBtTheme();

  // Ports list
  const parentPorts: ReactElement[] = [];
  const childrenPorts: ReactElement[] = [];
  const inputPorts: ReactElement[] = [];
  const outputPorts: ReactElement[] = [];

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
    <StyledNodeContainer
      className="bt-basic-node"
      borderColor={theme.btEditor.border}
      roundness={theme.btEditor.roundness}
      color={node.getColor()}
      selected={node.isSelected()}
      status={node.getExecStatus()}
      statusRunning={theme.btEditor.running}
      statusSuccess={theme.btEditor.success}
      statusFailure={theme.btEditor.failure}
      statusInvalid={theme.btEditor.invalid}
      shadowColor={theme.btEditor.shadow}
    >
      <StyledNodeSection className="bt-basic-layer">
        <StyledNodeDiagramPorts type="parent">
          {parentPorts.length > 0 ? (
            parentPorts
          ) : (
            <StyledNodePort color={"unset"} />
          )}
        </StyledNodeDiagramPorts>
        <StyledNodeTitle id="bt-node-title">{node.getName()}</StyledNodeTitle>
        <StyledNodeDiagramPorts type="children">
          {childrenPorts.length > 0 ? (
            childrenPorts
          ) : (
            <StyledNodePort color={"unset"} />
          )}
        </StyledNodeDiagramPorts>
      </StyledNodeSection>
      <StyledNodeSection className="bt-basic-layer">
        <StyledNodeTagPorts type="input">{inputPorts}</StyledNodeTagPorts>
        <StyledNodeTagPorts type="output">{outputPorts}</StyledNodeTagPorts>
      </StyledNodeSection>
    </StyledNodeContainer>
  );
};
