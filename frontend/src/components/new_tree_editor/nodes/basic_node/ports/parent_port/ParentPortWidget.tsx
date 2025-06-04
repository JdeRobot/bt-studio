import React from "react";
import { DiagramEngine, PortWidget } from "@projectstorm/react-diagrams";
import "./ParentPort.css";

export const ParentPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  return (
    <PortWidget port={port} engine={engine}>
      <div className="bt-port-parent"></div>
    </PortWidget>
  );
};
