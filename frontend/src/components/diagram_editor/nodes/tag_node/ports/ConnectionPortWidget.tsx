import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './ConnectionPort.css';

export const ConnectionPortWidget = ({ engine, port }: { engine: any, port: any }) => {

  // Determine the additional CSS class to use based on port type
  const getPortClass = () => {
    return port.options.type === 'children port' ? 'port-children' : 'port-parent';
  };

  const portClass = getPortClass();

  return (
    <PortWidget port={port} engine={engine} className={`port-widget ${portClass}`}>
    </PortWidget>
  );
};
