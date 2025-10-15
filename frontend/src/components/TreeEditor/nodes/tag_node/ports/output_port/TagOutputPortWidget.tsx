import React from "react";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import { useBtTheme } from "Contexts/BtThemeContext";
import {
  StyledNodeTagPort,
  StyledNodeTagPortContainer,
} from "Styles/TreeEditor/BTNode.styles";

export const OutputPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  const theme = useBtTheme();
  return (
    <StyledNodeTagPortContainer type="output">
      <StyledNodeTagPort
        port={port}
        engine={engine}
        color={theme.btEditor.border}
      />
    </StyledNodeTagPortContainer>
  );
};
