import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './TagOutputPort.css';

export const OutputPortWidget = ({ engine, port }: {engine: any, port: any }) => {
    return (
        <div className='tag-output-container'>
            <PortWidget port={port} engine={engine} className="tag-output-port">
            </PortWidget>
        </div>
    );
};
