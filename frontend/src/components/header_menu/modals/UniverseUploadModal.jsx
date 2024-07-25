import React, { useState, useEffect, useRef } from 'react';
import './UniverseUploadModal.css';
import Modal from '../../Modal/Modal';
import back_modal_img from '../../Modal/img/back.svg';
import close_modal_img from '../../Modal/img/close.svg';
import delete_icon from '../../diagram_editor/img/delete.svg';
import axios from 'axios';

const initialProjectData = {
  projectName: '',
};

const UniverseUploadModal = ({ onSubmit, isOpen, onClose, currentProject, openError}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialProjectData);
  const [existingUniverses, setUniversesProjects] = useState([]);
  const [uploadFile, setUploadFile] = useState("")

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

  const handleFileReader = (event) => {
    let reader = new FileReader();
    reader.readAsDataURL(event.target.files[0]);
    reader.onload = (e) => {
      setUploadFile(e.target.result)
    };
  }

  const saveZipUniverse = () => {
    console.log("Call the saving API");
  }

  return (
    <Modal id="universe-upload-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Upload a zip file with your universe</label>
          <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
        </div>
        <div className="form-row">
          <input
            onChange={handleFileReader}                
            type="file"
            accept=".zip,.rar,.7zip"
          />
        </div>
        <div className="form-row">
          <div className="project-modal-creation-buttons-container">
            <div className='project-modal-create-button' onClick={() => { saveZipUniverse(); } }>accept</div>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default UniverseUploadModal;