import React from "react";
import { DiagramEngine, PortWidget } from "@projectstorm/react-diagrams";
import "./TagInputPort.css";

export const InputPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  return (
    <div className="bt-tag-input-container">
      <PortWidget
        port={port}
        engine={engine}
        className="bt-tag-input-port"
      ></PortWidget>
    </div>
  );
};
