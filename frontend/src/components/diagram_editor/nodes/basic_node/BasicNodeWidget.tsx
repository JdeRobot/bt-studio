import React from 'react';
import { DefaultPortLabel } from '@projectstorm/react-diagrams';
import { SpecialPortWidget } from './ConnectionPortWidget';

import './BasicNode.css'

// The node widget controls the visualization of the custom node
export const SpecialNodeWidget = ({ engine, node }: { engine: any, node: any }) => {

    // Node style
    let nodeStyle: React.CSSProperties = {
        display: 'flex',
        justifyContent: "space-between",
        border: '1px solid black',
        borderRadius: '5px',
        background: node.getOptions().color || 'red',
        paddingTop: "10px",
        paddingBottom: "10px",
    };

    const titleStyle: React.CSSProperties = {
        fontWeight: 'bold'
    };

    // Ports list
    const parentPorts: JSX.Element[] = [];
    const childrenPorts: JSX.Element[] = [];
    const normalPorts: JSX.Element[] = [];

    // Get all the ports from the node and classify them
    Object.keys(node.getPorts()).forEach((portName) => {
        const port = node.getPort(portName);
        if (!port) return;

        if (port.options.type === 'parent port') {
            parentPorts.push(<SpecialPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'children port') {
            childrenPorts.push(<SpecialPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'data') {
            normalPorts.push(<DefaultPortLabel key={portName} engine={engine} port={port} />);
        }
    });

    // Apply styles depending on the quantity of ports (this is node styles)
    if (parentPorts.length > 0 && childrenPorts.length === 0) {
        nodeStyle = { ...nodeStyle, paddingRight: '10px' };
    } else if (parentPorts.length === 0 && childrenPorts.length > 0) {
        nodeStyle = { ...nodeStyle, paddingLeft: '10px' };
    } else {
        nodeStyle = { ...nodeStyle, paddingTop: '10px', paddingBottom: '10px' };
    }

    // Return the node to render
    return (
        <div className="node" style={nodeStyle}>
            <div className="parent-ports">
                {parentPorts}
                {normalPorts}
            </div>
            <div className="title" style={titleStyle}>
                {node.getOptions().name}
            </div>
            <div className="ports">
                {childrenPorts}
            </div>
        </div>
    );
};
