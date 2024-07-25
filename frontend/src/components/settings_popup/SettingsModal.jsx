import React, { useState, useEffect, useRef } from 'react';
import './SettingsModal.css';
import Modal from '../Modal/Modal';
import back_modal_img from '../Modal/img/back.svg'
import close_modal_img from '../Modal/img/close.svg'

const initialProjectData = {
  projectName: '',
};

const SettingsModal = ({ onSubmit, isOpen, onClose, currentProject, openError}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialProjectData);

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

  const handleCancel = () => {
    if (currentProject !== '') {
      onClose()
    }
  };

  const handleCreate = () => {
    if (formState.projectName === '') {
      return;
    }
    onClose()
  };

  return (
    <Modal id="universes-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Settings</label>
          <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
        </div>
        <div className="form-row">
            <ul className='project-entry-list'>
            </ul>
          </div>
      </form>
    </Modal>
  );
};

export default SettingsModal;