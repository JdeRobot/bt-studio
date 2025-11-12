import React from "react";
import { DiagramEngine, PortWidget } from "@projectstorm/react-diagrams";
import { useBtTheme } from "Contexts/BtThemeContext";
import { StyledNodePort } from "Styles/TreeEditor/BTNode.styles";

export const ChildrenPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  const theme = useBtTheme();
  return (
    <PortWidget port={port} engine={engine}>
      <StyledNodePort color={theme.btEditor.border} />
    </PortWidget>
  );
};
