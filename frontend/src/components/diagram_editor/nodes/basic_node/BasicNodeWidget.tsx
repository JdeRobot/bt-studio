import React from 'react';
import { DefaultPortLabel } from '@projectstorm/react-diagrams';
import { ConnectionPortWidget } from './ports/ConnectionPortWidget';
import { InputPortWidget } from './ports/InputPortWidget';
import { OutputPortWidget } from './ports/OutputPortWidget';

import './BasicNode.css'

// The node widget controls the visualization of the custom node
export const BasicNodeWidget = ({ engine, node }: { engine: any, node: any }) => {

    // Node style
    let nodeStyle: React.CSSProperties = {
        display: 'flex',
        justifyContent: "space-between",
        border: '1px solid black',
        borderRadius: '5px',
        background: node.getOptions().color || 'red',
        paddingTop: "10px",
        paddingBottom: "10px",
        flexDirection: "column"
    };

    // Ports list
    const parentPorts: JSX.Element[] = [];
    const childrenPorts: JSX.Element[] = [];
    const inputPorts: JSX.Element[] = [];
    const outputPorts: JSX.Element[] = [];

    // Get all the ports from the node and classify them
    Object.keys(node.getPorts()).forEach((portName) => {
        const port = node.getPort(portName);
        if (!port) return;

        if (port.options.type === 'parent port') {
            parentPorts.push(<ConnectionPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'children port') {
            childrenPorts.push(<ConnectionPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'input port') {
            inputPorts.push(<InputPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'output port') {
            outputPorts.push(<OutputPortWidget key={portName} engine={engine} port={port} />);
        }
    });

    // Apply styles depending on the quantity of ports (this is node styles)
    if (parentPorts.length > 0 && outputPorts.length === 0 && childrenPorts.length === 0) {
        nodeStyle = { ...nodeStyle, paddingRight: '10px' };
    } else if (parentPorts.length === 0 && childrenPorts.length > 0) {
        nodeStyle = { ...nodeStyle, paddingLeft: '10px' };
    } else {
        nodeStyle = { ...nodeStyle, paddingTop: '10px', paddingBottom: '10px' };
    }

    // Return the node to render
    return (
        <div className='basic-node' style={nodeStyle}>
            
            <div className='basic-layer'>
                {parentPorts}
                <div className="basic-title">
                    {node.getOptions().name}
                </div>
                {childrenPorts.length > 0 ? childrenPorts : <div className='placeholder'></div>}
            </div>
            <div className='basic-layer'>
                <div className="basic-left-ports">
                    {inputPorts}
                </div>
                <div className="basic-right-ports">
                    {outputPorts}
                </div>
            </div>
            
        </div>
    );
};
