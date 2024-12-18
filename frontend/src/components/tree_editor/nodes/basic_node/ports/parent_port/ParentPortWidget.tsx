import React from "react";
import { PortWidget } from "@projectstorm/react-diagrams";
import "./ParentPort.css";

export const ParentPortWidget = ({
  engine,
  port,
}: {
  engine: any;
  port: any;
}) => {
  return (
    <PortWidget port={port} engine={engine}>
      <div className="bt-port-parent"></div>
    </PortWidget>
  );
};
