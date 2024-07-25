import React, { useState, useEffect, useRef } from 'react';
import './UniverseModal.css';
import Modal from '../../Modal/Modal';
import back_modal_img from '../../Modal/img/back.svg';
import close_modal_img from '../../Modal/img/close.svg';
import delete_icon from '../../diagram_editor/img/delete.svg';
import axios from 'axios';
import UniverseUploadModal from './UniverseUploadModal';

const initialProjectData = {
  projectName: '',
};

const UniverseModal = ({ onSubmit, isOpen, onClose, currentProject, openError}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialProjectData);
  const [existingUniverses, setUniversesProjects] = useState([]);
  const [isUploadModalOpen, setUploadModalOpen] = useState(false);

  useEffect(() => {
    if (!isOpen) {
      return
    }

    if (focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    const listApiUrl = `/tree_api/get_universes_list?project_name=${currentProject}`;

    axios.get(listApiUrl)
      .then(response => {
        setUniversesProjects(response.data.universes_list)
      })
      .catch(error => {
        console.error('Error while fetching universes list:', error);
        openError(`An error occurred while fetching the universes list`);
      });
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

  const deleteUniverse = (universe_name) => {
    const apiUrl = `/tree_api/delete_universe?project_name=${currentProject}&universe_name=${universe_name}`;
    axios.get(apiUrl)
      .then(response => {
        if (response.data.success) {
        const listApiUrl = `/tree_api/get_universes_list?project_name=${currentProject}`;

        axios.get(listApiUrl)
          .then(response => {
            setUniversesProjects(response.data.universes_list)
          })
          .catch(error => {
            console.error('Error while fetching universes list:', error);
            openError(`An error occurred while fetching the universes list`);
          });
          console.log('Universe deleted successfully');
        } 
      })
      .catch(error => {
        if (error.response) {
          // The request was made and the server responded with a status code
          // that falls out of the range of 2xx
          if (error.response.status === 409) {
            openError(`The universe ${universe_name} does not exist`);
          } else {
            // Handle other statuses or general API errors
            openError('Unable to connect with the backend server. Please check the backend status.');
          }
        }
      });
  }

  const importFromZip = () => {
    console.log("Opening import modal");
    setUploadModalOpen(true);
  }

  const handleCloseUploadUniverseModal = (universe_name) => {
    setUploadModalOpen(false);
  };

  const handleFormSubmit = (data) => {
  }

  return (
    <Modal id="universes-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
        <UniverseUploadModal
          isOpen={isUploadModalOpen}
          onSubmit={handleFormSubmit}
          onClose={handleCloseUploadUniverseModal}
          openError={openError}
        />
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Manage your Universes</label>
          <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
        </div>
        <div className="form-row">
              <ul className='project-entry-list'>
                {Object.entries(existingUniverses).map((project) => {
                  return (
                    <div className='project-entry' onClick={() => onClose(project[1])}>
                      <label className='project-entry-name'>{project[1]}</label>
                      <img
                        className="project-entry-delete icon"
                        style={{ color: 'white' }}
                        title='Delete'
                        onClick={(e) => { deleteUniverse(project[1]); e.stopPropagation(); } }
                        src={delete_icon}>
                      </img>
                    </div>
                  );
                })}
              </ul>
            </div>
        <div className="form-row">
          <div className="project-modal-creation-buttons-container">
            <div className='project-modal-create-button' onClick={() => { importFromZip(); } }>Import from zip</div>
            <div className='project-modal-create-button' onClick={() => {} }>Import from Robotics Backend library</div>
            {/* <div className='project-modal-create-button'>Other</div> */}
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default UniverseModal;