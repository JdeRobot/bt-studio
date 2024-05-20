import React, { useState, useEffect, useRef } from 'react';
import { BasicNodeWidget } from './nodes/basic_node/BasicNodeWidget.tsx';

import { Saturation, Hue, ColorPicker, useColor } from "react-color-palette";
import "react-color-palette/css";

import './EditActionModal.css';
import Modal from '../Modal/Modal';

const EditActionModal = ({ isOpen, onClose, currentActionNode, engine, addInputPort}) => {
  // const [color, setColor] = useColor(currentActionNode ? currentActionNode.getColor().replaceAll(",", " ") : "rgb(128 0 128)");
  const [color, setColor] = useColor("rgb(128 0 128)");

  useEffect(() => {
    if (currentActionNode) {
      currentActionNode.setColor('rgb('+Math.round(color.rgb['r'])+','+Math.round(color.rgb['g'])+','+Math.round(color.rgb['b'])+')');
    }
  }, [color]);

  return (
    <Modal hasCloseBtn={true} isOpen={isOpen} onClose={() => onClose(color)}>
      <div className="form-row">
        <label htmlFor="actionNameEditor">Action Editor</label>
      </div>
      <div className="form-row">
        {currentActionNode && 
          <BasicNodeWidget
            engine={engine}
            node={currentActionNode} 
          />
        }
      </div>
      <div className="form-row">
        <button className="menu-button" onClick={() => addInputPort(currentActionNode)} title='Add input to action'>Add input</button>
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