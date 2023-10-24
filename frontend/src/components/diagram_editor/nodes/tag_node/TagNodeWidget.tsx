import React from 'react';
import { DefaultPortLabel } from '@projectstorm/react-diagrams';
import { ConnectionPortWidget } from './ports/ConnectionPortWidget';

import './TagNode.css'

// The node widget controls the visualization of the custom node
export const TagNodeWidget = ({ engine, node }: { engine: any, node: any }) => {

    // Ports list
    const parentPorts: JSX.Element[] = [];
    const childrenPorts: JSX.Element[] = [];

    // Initial class
    let nodeClass = "tag-node basic";

    // Get all the ports from the node and classify them
    Object.keys(node.getPorts()).forEach((portName) => {
        const port = node.getPort(portName);
        if (!port) return;

        if (port.options.type === 'parent port') {
            parentPorts.push(<ConnectionPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'children port') {
            childrenPorts.push(<ConnectionPortWidget key={portName} engine={engine} port={port} />);
        }
    });

    // Adjust class name depending on the quantity of ports
    if (parentPorts.length > 0 && childrenPorts.length === 0) {
        nodeClass = "tag-node parent-only";
    } else if (parentPorts.length === 0 && childrenPorts.length > 0) {
        nodeClass = "tag-node children-only";
    }

    // Return the node to render
    return (
        <div className={nodeClass}>
            
            <div className='layer'>
                {parentPorts}
                <div className="title">
                    {node.getOptions().name}
                </div>
                {childrenPorts.length > 0 ? childrenPorts : <div className='placeholder'></div>}
            </div>
            
        </div>
    );
};
