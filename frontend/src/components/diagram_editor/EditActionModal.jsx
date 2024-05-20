import React, { useState, useEffect, useRef } from 'react';
import { BasicNodeWidget } from './nodes/basic_node/BasicNodeWidget.tsx';

import './EditActionModal.css';
import Modal from '../Modal/Modal';

const EditActionModal = ({ isOpen, onClose, currentActionNode, engine, addInputPort}) => {

  return (
    <Modal hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
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
    </Modal>
  );
};

export default EditActionModal;