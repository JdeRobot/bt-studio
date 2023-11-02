import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './InputPort.css';

export const InputPortWidget = ({ engine, port }: {engine: any, port: any }) => {
    return (
        <div className='input-container'>
            <PortWidget port={port} engine={engine} className="input-port-widget">
            </PortWidget>
            <div className='input-label'>{port.options.name}</div>
        </div>


    );
};
