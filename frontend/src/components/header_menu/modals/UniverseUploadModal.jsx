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

  const [formState, setFormState] = useState(initialProjectData);
  const [uploadedUniverse, setUploadedUniverse] = useState("");
  const [uploadStatus, setUploadStatus] = useState("");
  const [uploadPercentage, setUploadPercentage] = useState(0);
  const [universeName, setUniverseName] = useState("");

  const handleCancel = () => {
    if (currentProject !== '') {
      onClose()
    }
  };

  const handleFileReader = (event) => {

    setUploadStatus('Uploading');
    let reader = new FileReader();
    let file = event.target.files[0];
    let fileName = file.name.toString().split(".")[0];

    reader.readAsDataURL(file);

    reader.onloadstart = () => {
      setUploadPercentage(0);
    };

    reader.onprogress = (data) => {
      if (data.lengthComputable) {
        const progress = Math.round((data.loaded / data.total) * 100);
        console.log(progress); 
        setUploadPercentage(progress);
      }
    };

    reader.onload = (e) => {
      console.log("Loaded!");
      const base64String = e.target.result.split(',')[1]; // Remove the data URL prefix
      setUploadedUniverse(base64String);
      setUniverseName(fileName);
      setUploadStatus('Uploaded');
      setUploadPercentage(100);
    };

    reader.onerror = () => {
      setUploadStatus('Error');
      setUploadPercentage(0);
    };
  }

  const saveZipUniverse = () => {
    
    console.log("Call the saving API");

    if (uploadPercentage != 100){
      console.warn("Not yet uploaded!");
      return;
    }

    axios.post('/tree_api/upload_universe/', {
      universe_name: universeName,
      zip_file: uploadedUniverse,
      app_name: currentProject,
    })
    .then(response => {
      if (response.data.success) {
        console.log('Universe saved successfully.');
      } else {
        console.error('Error saving project:', response.data.message || 'Unknown error');
      }
    })
    .catch(error => {
      console.error('Axios Error:', error);
    });

    onClose();
  }

  return (
    <Modal id="universe-upload-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Upload a zip file with your universe</label>
          <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
        </div>
        <div className="form-row">
          <div className="modal-complex-input-row-container">
            <input className='modal-complex-input'
              onChange={handleFileReader}                
              type="file"
              accept=".zip,.rar,.7zip"
            />
          </div>
        </div>
        <div className='upload-percentage'>
          {uploadPercentage}
        </div>
        <div className="form-row">
          <div className="project-modal-creation-buttons-container">
            <div className='project-modal-create-button' onClick={() => { saveZipUniverse(); } }>Accept</div>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default UniverseUploadModal;