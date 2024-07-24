import React, { useState, useEffect, useRef } from 'react';
import './UniverseModal.css';
import Modal from '../../Modal/Modal';
import back_modal_img from '../../Modal/img/back.svg'
import close_modal_img from '../../Modal/img/close.svg'
import axios from 'axios';

const initialProjectData = {
  projectName: '',
};

var universe_config = {
  "name": "follow_person_ros2",
  "launch_file_path": "/opt/jderobot/Launchers/follow_person.launch.py",
  "ros_version": "ROS2",
  "visualization": "gazebo_rae",
  "world": "gazebo",
  "template": "RoboticsAcademy/exercises/static/exercises/follow_person_newmanager/python_template/",
  "exercise_id": "follow_person_newmanager"
}

const UniverseModal = ({ onSubmit, isOpen, onClose, currentProject, openError}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialProjectData);

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
        console.log(response.data.universes_list)
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

  return (
    <Modal id="universes-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Manage your Universes</label>
          <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
        </div>
        <div className="form-row">
            <ul className='project-entry-list'>
            </ul>
          </div>
          <div className="form-row">
            <div className="project-modal-creation-buttons-container">
              <div className='project-modal-create-button' onClick={() => {} }>Import Universe</div>
              <div className='project-modal-create-button'>Other</div>
              <div className='project-modal-create-button'>Other</div>
            </div>
          </div>
      </form>
    </Modal>
  );
};

export default UniverseModal;