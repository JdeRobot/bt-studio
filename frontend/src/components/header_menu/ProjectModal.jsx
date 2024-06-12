import React, { useState, useEffect, useRef } from 'react';
import './ProjectModal.css';
import Modal from '../Modal/Modal';
import axios from 'axios';

const ProjectModal = ({ onSubmit, isOpen, onClose, existingProjects, setExistingProjects}) => {
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

  const handleCancel = (event) => {
  };

  return (
    <Modal id="project-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="form-row">
          <label htmlFor="actionName" style={{textAlign: "center"}}>Open a Project</label>
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
              <div className='project-entry' onClick={() => onClose(project[1])}>{project[1]}</div>
            )
          })}
          </ul>
        </div>
      </form>
    </Modal>
  );
};

export default ProjectModal;