import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './OutputPort.css';

export const OutputPortWidget = ({ engine, port }: {engine: any, port: any }) => {
    return (
        <div className='output-container'>
            <PortWidget port={port} engine={engine} className="output-port-widget">
            </PortWidget>
        </div>
    );
};
