import React, { useState, useEffect, useRef } from 'react';
import { BasicNodeWidget } from './nodes/basic_node/BasicNodeWidget.tsx';

import { Saturation, Hue, ColorPicker, useColor } from "react-color-palette";
import "react-color-palette/css";

import './EditActionModal.css';
import Modal from '../Modal/Modal';
import { OutputPortModel } from './nodes/basic_node/ports/output_port/OutputPortModel';
import { InputPortModel } from './nodes/basic_node/ports/input_port/InputPortModel';

const EditActionModal = ({ isOpen, onClose, currentActionNode, addInputPort, addOutputPort}) => {
  // const [color, setColor] = useColor(currentActionNode ? currentActionNode.getColor().replaceAll(",", " ") : "rgb(128 0 128)");
  const [color, setColor] = useColor("rgb(128 0 128)");
  const [nodePorts, setNodePorts] = useState(currentActionNode ? currentActionNode.getPorts() : []);
  const [, updateState] = React.useState();
  const forceUpdate = React.useCallback(() => updateState({}), []);

  useEffect(() => {
    if (currentActionNode) {
      currentActionNode.setColor('rgb('+Math.round(color.rgb['r'])+','+Math.round(color.rgb['g'])+','+Math.round(color.rgb['b'])+')');
    }
  }, [color]);

  const isBackgroundDark = () => {
    return ((color.rgb['r'] + color.rgb['g'] + color.rgb['b']) / 3) < 123
  }

  return (
    <Modal hasCloseBtn={true} isOpen={isOpen} onClose={() => onClose(color)}>
      <div className="form-row">
        <label htmlFor="actionNameEditor">Action Editor</label>
      </div>
      <div className="form-row">
        {currentActionNode &&
          <div className="node-editor" style={{backgroundColor: currentActionNode.getColor()}}>
            <label className="node-editor-name" style={{color: isBackgroundDark() ? 'white' : 'black'}}>{currentActionNode.getName()}</label>
            <div className="node-editor-io">
              <div className="node-editor-inputs">
                {Object.entries(currentActionNode.getPorts()).map((port, index) => {
                  if (port[1] instanceof InputPortModel) {
                    return (
                      <div key={index} className="node-editor-input" style={{color: isBackgroundDark() ? 'white' : 'black'}}>
                        {port[0]}
                      </div>
                    );
                  }
                })}
                <button 
                  className="node-editor-button" 
                  style={{color: isBackgroundDark() ? 'white' : 'black'}} 
                  onClick={() => {addInputPort(); forceUpdate();}} 
                  title='Add input'>
                  +
                </button>
              </div>
              <div className="node-editor-outputs">
                {Object.entries(currentActionNode.getPorts()).map((port, index) => {
                  if (port[1] instanceof OutputPortModel) {
                    return (
                      <div key={index} className="node-editor-output" style={{color: isBackgroundDark() ? 'white' : 'black'}}>
                        {port[0]}
                      </div>
                    );
                  }
                })}
                <button 
                  className="node-editor-button" 
                  style={{color: isBackgroundDark() ? 'white' : 'black'}} 
                  onClick={() => {addOutputPort(); forceUpdate();}} 
                  title='Add output'>
                  +
                </button>
              </div>
            </div>
          </div>
        }
      </div>
      <div className="form-row">
        <label for="favcolor">Color:</label>
        <Saturation height={50} width={300} color={color} onChange={setColor} />
        <Hue color={color} onChange={setColor} />
      </div>
    </Modal>
  );
};

export default EditActionModal;