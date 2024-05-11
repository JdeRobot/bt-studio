import React, { useState, useEffect, useRef } from 'react';
import { BasicNodeWidget } from './nodes/basic_node/BasicNodeWidget.tsx';

import './EditActionModal.css';
import Modal from '../Modal/Modal';

const initialEditActionModalData = {
  actionNameEditor: ''
};

const EditActionModal = ({ isOpen, onClose, currentActionNode, engine}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialEditActionModalData);

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
        <label htmlFor="actionNameEditor">Action Name</label>
        <input
            ref={focusInputRef}
            type="text"
            id="actionNameEditor"
            name="actionNameEditor"
            onChange={handleInputChange}
            autoComplete='off'
            required
          />
      </div>
      <div className="form-row">
        <label htmlFor="actionNameEditor">Action Node</label>
        {currentActionNode && 
          <BasicNodeWidget
            engine={engine}
            node={currentActionNode} 
          />
        }
      </div>
    </Modal>
  );
};

export default EditActionModal;