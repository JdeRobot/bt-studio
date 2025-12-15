import React from "react";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import {
  StyledNodeTagPort,
  StyledNodeTagPortContainer,
  StyledNodeTagPortLabel,
} from "BtStyles/TreeEditor/BTNode.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";

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
      <StyledNodeTagPortLabel type="output">
        {port.options.name}
      </StyledNodeTagPortLabel>
      <StyledNodeTagPort
        port={port}
        engine={engine}
        color={theme.btEditor.border}
      />
    </StyledNodeTagPortContainer>
  );
};
