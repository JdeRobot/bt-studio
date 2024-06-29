import React, { useState, useEffect, useRef } from 'react';
import './ProjectModal.css';
import Modal from '../Modal/Modal';
import back_modal_img from '../Modal/img/back.svg'
import close_modal_img from '../Modal/img/close.svg'
import delete_icon from '../diagram_editor/img/delete.svg';
import axios from 'axios';

const initialProjectData = {
  projectName: '',
};

const ProjectModal = ({ onSubmit, isOpen, onClose, currentProject, existingProjects, setExistingProjects, createProject, openError}) => {
  const focusInputRef = useRef(null);
  const [createProjectOpen, setCreateProjectOpen] = useState(false);
  const [formState, setFormState] = useState(initialProjectData);

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
        openError(`An error occurred while fetching the project list`);
      });
    setFormState(initialProjectData)
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

  const handleSubmit = (event) => {
  };

  const handleCancel = () => {
    setCreateProjectOpen(false);
    if (currentProject !== '') {
      onClose()
    }
  };

  const handleCreate = () => {
    if (formState.projectName === '') {
      return;
    }
    setCreateProjectOpen(false);
    createProject(formState.projectName);
    onClose()
  };

  const deleteProject = (project) => {
    if (currentProject === project) {
      //TODO: change this to change project before deleting
      return;
    }
    const apiUrl = `/tree_api/delete_project?project_name=${encodeURIComponent(project)}`;
    axios.get(apiUrl)
      .then(response => {
        if (response.data.success) {
          const listApiUrl = `/tree_api/get_project_list`;
  
          axios.get(listApiUrl)
            .then(response => {
              setExistingProjects(response.data.project_list)
            })
            .catch(error => {
              console.error('Error while fetching project list:', error);
              openError(`An error occurred while fetching the project list`);
            });
          console.log('Project deleted successfully');
        } 
      })
      .catch(error => {
        if (error.response) {
          // The request was made and the server responded with a status code
          // that falls out of the range of 2xx
          if (error.response.status === 409) {
            openError(`The project ${project} does not exist`);
          } else {
            // Handle other statuses or general API errors
            openError('Unable to connect with the backend server. Please check the backend status.');
          }
        }
      });
  }

  return (
    <Modal id="project-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        { !createProjectOpen ? (
          <>
          <div className="modal-titlebar">
            <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Open a Project</label>
            <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
          </div>
          <div className="form-row">
              <ul className='project-entry-list'>
                {Object.entries(existingProjects).map((project) => {
                  return (
                    <div className='project-entry' onClick={() => onClose(project[1])}>
                      <label className='project-entry-name'>{project[1]}</label>
                      <img
                        className="project-entry-delete icon"
                        style={{ color: 'white' }}
                        title='Delete'
                        onClick={(e) => { deleteProject(project[1]); e.stopPropagation(); } }
                        src={delete_icon}>
                      </img>
                    </div>
                  );
                })}
              </ul>
            </div>
            <div className="form-row">
              <div className="project-modal-creation-buttons-container">
                <div className='project-modal-create-button' onClick={() => { setCreateProjectOpen(true) } }>Create New Project</div>
                {/* <div className='project-modal-create-button'>Other</div>
                <div className='project-modal-create-button'>Other</div> */}
              </div>
            </div>
            </>
        ) : (
          <>
          <div className="modal-titlebar">
            <img className="modal-titlebar-back" title='Open a Project' onClick={() => { setCreateProjectOpen(false); } } src={back_modal_img}></img>
            <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Create New Project</label>
            <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
          </div>
          <div className="modal-complex-input-row-container">
            <div className="project-create-name modal-complex-input-container">
              <input
                ref={focusInputRef}
                type="text"
                id="projectName"
                name="projectName"
                className='modal-complex-input'
                onChange={handleInputChange}
                autoComplete='off'
                placeholder="Project Name"
              />
              <label for="projectName" class="modal-complex-input-label">Project Name</label>
              <label for="projectName" class="modal-complex-input-indications">
                A unique name that is used for the project folder and other resources. The name should be in lower case without spaces and should not start with a number. 
              </label>
            </div>
          </div>
          <div className="modal-complex-input-row-container">
            <div id="create-new-project" onClick={() => handleCreate()}>Create Project</div>
          </div>
          </>
        )
        }
      </form>
    </Modal>
  );
};

export default ProjectModal;