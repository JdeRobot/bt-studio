import React from 'react';
import { DefaultPortLabel } from '@projectstorm/react-diagrams';
import { SpecialPortWidget } from './SpecialPortWidget';

export const SpecialNodeWidget = ({ engine, node }: { engine: any, node: any }) => {
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

    const parentPorts: JSX.Element[] = [];
    const childrenPorts: JSX.Element[] = [];
    const normalPorts: JSX.Element[] = [];

    Object.keys(node.getPorts()).forEach((portName) => {
        const port = node.getPort(portName);
        if (!port) return;

        if (port.options.type === 'parent port') {
            parentPorts.push(<SpecialPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'children port') {
            childrenPorts.push(<SpecialPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'normal') {
            normalPorts.push(<DefaultPortLabel key={portName} engine={engine} port={port} />);
        }
    });

    if (parentPorts.length > 0 && childrenPorts.length === 0) {
        nodeStyle = { ...nodeStyle, paddingRight: '10px' };
    } else if (parentPorts.length === 0 && childrenPorts.length > 0) {
        nodeStyle = { ...nodeStyle, paddingLeft: '10px' };
    } else {
        nodeStyle = { ...nodeStyle, paddingTop: '10px', paddingBottom: '10px' };
    }

    return (
        <div className="node" style={nodeStyle}>
            <div className="parent-ports">
                {parentPorts}
            </div>
            <div className="title" style={titleStyle}>
                {node.getOptions().name}
            </div>
            <div className="ports">
                {childrenPorts}
                {normalPorts}
            </div>
        </div>
    );
};
