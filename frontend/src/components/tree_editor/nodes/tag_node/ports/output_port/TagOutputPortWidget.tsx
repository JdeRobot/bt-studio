import React from "react";
import { DiagramEngine, PortWidget } from "@projectstorm/react-diagrams";
import "./TagOutputPort.css";

export const OutputPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  return (
    <div className="bt-tag-output-container">
      <PortWidget
        port={port}
        engine={engine}
        className="bt-tag-output-port"
      ></PortWidget>
    </div>
  );
};
