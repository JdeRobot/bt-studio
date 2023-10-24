import React, {useState } from 'react';
import { DefaultPortLabel } from '@projectstorm/react-diagrams';
import { InputPortWidget } from './ports/InputPortWidget';
import { OutputPortWidget } from './ports/OutputPortWidget';

import './TagNode.css'

// The node widget controls the visualization of the custom node
export const TagNodeWidget = ({ engine, node }: { engine: any, node: any }) => {

    // State to store and update the text
    const [nodeText, setNodeText] = useState("value");

    // Ports list
    const inputPorts: JSX.Element[] = [];
    const outputPorts: JSX.Element[] = [];

    // Initial class
    let nodeClass = "tag-node basic";

    // Get all the ports from the node and classify them
    Object.keys(node.getPorts()).forEach((portName) => {
        const port = node.getPort(portName);
        if (!port) return;

        if (port.options.type === 'input port') {
            inputPorts.push(<InputPortWidget key={portName} engine={engine} port={port} />);
        } else if (port.options.type === 'output port') {
            outputPorts.push(<OutputPortWidget key={portName} engine={engine} port={port} />);
        }
    });

    // Adjust class name depending on the quantity of ports
    if (inputPorts.length > 0 && outputPorts.length === 0) {
        nodeClass = "tag-node input";
    } else if (inputPorts.length === 0 && outputPorts.length > 0) {
        nodeClass = "tag-node output";
    }

    const showPromptAndUpdateText = () => {
        const newText = prompt("Enter new text:", nodeText);
        if (newText !== null) {
            if (newText.length <= 25) {
                setNodeText(newText);
            } else {
                alert("Text should be less than or equal to 25 characters.");
            }
        }
    };

    // Return the node to render
    return (
        <div className={nodeClass}>
            <div className='layer'>
                {inputPorts}
                <div onDoubleClick={showPromptAndUpdateText}>
                    {nodeText}
                </div>
                {outputPorts}
            </div>
        </div>
    );
};
