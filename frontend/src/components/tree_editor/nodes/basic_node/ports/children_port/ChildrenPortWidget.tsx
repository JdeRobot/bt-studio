import React from "react";
import { PortWidget } from "@projectstorm/react-diagrams";
import "./ChildrenPort.css";

export const ChildrenPortWidget = ({
  engine,
  port,
}: {
  engine: any;
  port: any;
}) => {
  return (
    <PortWidget port={port} engine={engine}>
      <div className="bt-port-children"></div>
    </PortWidget>
  );
};
