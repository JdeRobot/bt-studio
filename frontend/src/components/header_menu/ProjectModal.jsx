import React, { useState, useEffect, useRef } from 'react';
import './ProjectModal.css';
import Modal from '../Modal/Modal';
import close_modal_img from '../Modal/img/close.svg'
import delete_icon from '../diagram_editor/img/delete.svg';
import axios from 'axios';

const ProjectModal = ({ onSubmit, isOpen, onClose, currentProject, existingProjects, setExistingProjects, createProject}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState("");

  const onOptionChange = e => {
    handleInputChange(e)
  }

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
    const listApiUrl = `/tree_api/get_project_list`;
  
    axios.get(listApiUrl)
      .then(response => {
        setExistingProjects(response.data.project_list)
      })
      .catch(error => {
        console.error('Error while fetching project list:', error);
        window.alert(`An error occurred while fetching the project list`);
      });
  }, [isOpen]);

  const handleInputChange = (event) => {
  };

  const handleSubmit = (event) => {
  };

  const handleCancel = () => {
    if (currentProject !== '') {
      onClose()
    }
  };

  return (
    <Modal id="project-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{textAlign: "center"}}>Open a Project</label>
          <img className="modal-titlebar-close" onClick={() => {handleCancel()}} src={close_modal_img}></img>
        </div>
        <div className="form-row">
          {/* <input
            ref={focusInputRef}
            type="text"
            id="actionName"
            name="actionName"
            onChange={handleInputChange}
            autoComplete='off'
          /> */}
          <ul className='project-entry-list'>
          {Object.entries(existingProjects).map((project) => {
            return (
              <div className='project-entry' onClick={() => onClose(project[1])}>
                <label className='project-entry-name'>{project[1]}</label>
                <img
                  className="project-entry-delete icon"
                  style={{color: 'white'}}
                  title='Delete'
                  onClick={(e) => {console.log("delete");e.stopPropagation();}}
                  src={delete_icon}>
                </img>
              </div>
            )
          })}
          </ul>
        </div>
        <div className="form-row">
          <div className="project-modal-creation-buttons-container">
            <div className='project-modal-create-button' onClick={() => {createProject(); onClose()}}>Create New Project</div>
            <div className='project-modal-create-button'>Other</div>
            <div className='project-modal-create-button'>Other</div>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default ProjectModal;