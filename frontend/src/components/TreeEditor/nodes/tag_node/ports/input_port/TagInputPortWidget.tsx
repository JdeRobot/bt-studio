import React from "react";
import { DiagramEngine, PortWidget } from "@projectstorm/react-diagrams";
import { useBtTheme } from "Contexts/BtThemeContext";
import { StyledNodeTagPort, StyledNodeTagPortContainer } from "Styles/TreeEditor/BTNode.styles";

export const InputPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  const theme = useBtTheme();
  return (
    <StyledNodeTagPortContainer type="input">
      <StyledNodeTagPort
        port={port}
        engine={engine}
        color={theme.btEditor.border}
      />
    </StyledNodeTagPortContainer>
  );
};
