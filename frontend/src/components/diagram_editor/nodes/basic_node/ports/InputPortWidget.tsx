import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './InputPort.css';

export const InputPortWidget = ({ engine, port }: {engine: any, port: any }) => {
    return (
        <div className='container-style'>
            <PortWidget port={port} engine={engine} className="input-port-widget">
            </PortWidget>
            <div className='label-style'>{port.options.name}</div>
        </div>


    );
};
