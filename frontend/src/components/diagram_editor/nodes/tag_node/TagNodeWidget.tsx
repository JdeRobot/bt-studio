import React, {useState, useEffect } from 'react';
import { DefaultPortLabel } from '@projectstorm/react-diagrams';
import { InputPortWidget } from './ports/input_port/TagInputPortWidget';
import { OutputPortWidget } from './ports/output_port/TagOutputPortWidget';

import './TagNode.css'

// The node widget controls the visualization of the custom node
export const TagNodeWidget = ({ engine, node }: { engine: any, node: any }) => {

    // Tag style
    let tagStyle: React.CSSProperties = {
        display: 'flex',
        justifyContent: "space-between",
        border: node.getBorderRadius() + 'px solid ' +  node.getBorderColor(),
        background: node.isFromBlackboard() ? 'black' : 'gray',
        paddingTop: '5px',
        paddingBottom: '5px',
        flexDirection: "column"
    };

    // Ports list
    const inputPorts: JSX.Element[] = [];
    const outputPorts: JSX.Element[] = [];

    // Initial class
    let nodeClass = "tag-node";

    // Get all the ports from the node and classify them
    Object.keys(node.getPorts()).forEach((portName) => {
        const port = node.getPort(portName);
        if (!port) return;

        if (port.options.type === 'tag input') {
            inputPorts.push(<InputPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'tag output') {
            outputPorts.push(<OutputPortWidget key={portName} engine={engine} port={port} />);
        }
    });

    // Adjust class name depending on the quantity of ports
    if (inputPorts.length > 0 && outputPorts.length === 0) {
        nodeClass = "tag-node tag-input";
    } else if (inputPorts.length === 0 && outputPorts.length > 0) {
        nodeClass = "tag-node tag-output";
    }

    // Return the node to render
    return (
        <div className={nodeClass} style={tagStyle}>
            <div className='tag-layer'>
                {inputPorts}
                {node.getName()}
                {outputPorts}
            </div>
        </div>
    );
};
