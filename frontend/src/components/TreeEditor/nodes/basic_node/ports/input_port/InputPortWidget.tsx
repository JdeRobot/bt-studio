import React from "react";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import {
  StyledNodeTagPort,
  StyledNodeTagPortContainer,
  StyledNodeTagPortLabel,
} from "Styles/TreeEditor/BTNode.styles";
import { useBtTheme } from "Contexts/BtThemeContext";

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
      <StyledNodeTagPortLabel type="input">
        {port.options.name}
      </StyledNodeTagPortLabel>
    </StyledNodeTagPortContainer>
  );
};
