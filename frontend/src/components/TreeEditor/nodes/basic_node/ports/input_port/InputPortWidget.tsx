import React from "react";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import {
  StyledNodeTagPort,
  StyledNodeTagPortContainer,
  StyledNodeTagPortLabel,
} from "BtStyles/TreeEditor/BTNode.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";

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
