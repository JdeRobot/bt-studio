import React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams';
import './TagInputPort.css';

export const InputPortWidget = ({ engine, port }: {engine: any, port: any }) => {
    return (
        <div className='tag-input-container'>
            <PortWidget port={port} engine={engine} className="tag-input-port">
            </PortWidget>
        </div>


    );
};
