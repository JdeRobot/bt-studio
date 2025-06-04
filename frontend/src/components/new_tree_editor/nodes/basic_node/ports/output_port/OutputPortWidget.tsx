import React from "react";
import { DiagramEngine, PortWidget } from "@projectstorm/react-diagrams";
import "./OutputPort.css";

export const OutputPortWidget = ({
  engine,
  port,
}: {
  engine: DiagramEngine;
  port: any;
}) => {
  return (
    <div className="bt-output-container">
      <div className="bt-output-label">{port.options.name}</div>
      <PortWidget
        port={port}
        engine={engine}
        className="bt-output-port-widget"
      ></PortWidget>
    </div>
  );
};
