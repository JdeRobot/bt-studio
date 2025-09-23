import React from "react";
import { DiagramEngine} from "@projectstorm/react-diagrams";
import {
  StyledNodeTagPort,
  StyledNodeTagPortContainer,
  StyledNodeTagPortLabel,
} from "Styles/TreeEditor/BTNode.styles";
import { useBtTheme } from "Contexts/BtThemeContext";

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
