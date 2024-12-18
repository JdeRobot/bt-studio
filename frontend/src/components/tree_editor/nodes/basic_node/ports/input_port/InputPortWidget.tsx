import React from "react";
import { PortWidget } from "@projectstorm/react-diagrams";
import "./InputPort.css";

export const InputPortWidget = ({
  engine,
  port,
}: {
  engine: any;
  port: any;
}) => {
  return (
    <div className="bt-input-container">
      <PortWidget
        port={port}
        engine={engine}
        className="bt-input-port-widget"
      ></PortWidget>
      <div className="bt-input-label">{port.options.name}</div>
    </div>
  );
};
