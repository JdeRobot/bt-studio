import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './ParentPort.css';

export const ParentPortWidget = ({ engine, port }: { engine: any, port: any }) => {

  return (
    <PortWidget port={port} engine={engine} className="port-parent">
    </PortWidget>
  );
};
