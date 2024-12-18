import React from "react";
import { PortWidget } from "@projectstorm/react-diagrams";
import "./TagOutputPort.css";

export const OutputPortWidget = ({
  engine,
  port,
}: {
  engine: any;
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
