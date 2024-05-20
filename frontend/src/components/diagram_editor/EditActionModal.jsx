import React, { useState, useEffect, useRef } from 'react';
import { BasicNodeWidget } from './nodes/basic_node/BasicNodeWidget.tsx';

import './EditActionModal.css';
import Modal from '../Modal/Modal';

const initialEditActionModalData = {
  actionNameEditor: ''
};

const EditActionModal = ({ isOpen, onClose, currentActionNode, addInputPort, orig_engine}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialEditActionModalData);
  const engine = orig_engine;

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

  return (
    <Modal hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <div className="form-row">
        <label htmlFor="actionNameEditor">Edit Action Node</label>
        {currentActionNode !== "" &&
          <label>{currentActionNode}</label>
        }
      </div>
      <div className="form-row">
        <button className="menu-button" onClick={() => addInputPort()} title='Add input to action'>Click</button>
      </div>
    </Modal>
  );
};

export default EditActionModal;