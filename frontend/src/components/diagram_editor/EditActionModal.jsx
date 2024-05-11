import React, { useState, useEffect, useRef } from 'react';
import './EditActionModal.css';
import Modal from '../Modal/Modal';

const initialEditActionModalData = {
  actionName: '',
  templateType: "empty",
  allowCreation: false,
};

function CreateButton({ hasToDisplay }) {
  if (hasToDisplay) {
    return <button type="submit" id="create-new-action">Create</button>;
  }
  return null;
}

const EditActionModal = ({ isOpen, onClose}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialEditActionModalData);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  return (
    <Modal hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <div className="form-row">
        <label htmlFor="actionName">Action Name</label>
      </div>
    </Modal>
  );
};

export default EditActionModal;