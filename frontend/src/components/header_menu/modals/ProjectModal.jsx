import React, { useState, useEffect, useRef } from "react";
import "./ProjectModal.css";
import Modal from "../../Modal/Modal";
import { ReactComponent as BackIcon } from "../../Modal/img/back.svg";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";
import { ReactComponent as DeleteIcon } from "../../diagram_editor/img/delete.svg";
import axios from "axios";

const initialProjectData = {
  projectName: "",
};

const ProjectModal = ({
  onSubmit,
  isOpen,
  onClose,
  currentProject,
  existingProjects,
  setExistingProjects,
  createProject,
  openError,
}) => {
  const focusInputRef = useRef(null);
  const [createProjectOpen, setCreateProjectOpen] = useState(false);
  const [formState, setFormState] = useState(initialProjectData);

  const getProjects = async () => {
    const listApiUrl = `/tree_api/get_project_list`;
    try {
      const response = await axios.get(listApiUrl);
      setExistingProjects(response.data.project_list);
      setFormState(initialProjectData);
    } catch (error) {
      console.error("Error while fetching project list:", error);
      openError(`An error occurred while fetching the project list`);
    }
  };

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    getProjects();
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

  const handleSubmit = (event) => {};

  const handleCancel = () => {
    setCreateProjectOpen(false);
    if (currentProject !== "") {
      onClose();
    }
  };

  const handleCreate = () => {
    if (formState.projectName === "") {
      return;
    }
    setCreateProjectOpen(false);
    createProject(formState.projectName);
    onClose();
  };

  const deleteProject = async (project) => {
    if (currentProject === project) {
      //TODO: change this to change project before deleting
      return;
    }
    const apiUrl = `/tree_api/delete_project?project_name=${encodeURIComponent(project)}`;
    const listApiUrl = `/tree_api/get_project_list`;

    // Delete and update
    const response = await axios.get(apiUrl);
    try {
      if (response.data.success) {
        await getProjects();
      }
      console.log("Project deleted successfully");
    } catch (error) {
      console.error("Error while fetching project list:", error);
      openError(`An error occurred while fetching the project list`);
    }
  };

  return (
    <Modal
      id="project-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={onSubmit} onReset={handleCancel}>
        {!createProjectOpen ? (
          <>
            <div className="modal-titlebar">
              <label
                className="modal-titlebar-title"
                htmlFor="actionName"
                style={{ textAlign: "center" }}
              >
                Open a Project
              </label>
              <CloseIcon
                className="modal-titlebar-close icon"
                onClick={() => {
                  handleCancel();
                }}
                fill={"var(--icon)"}
              />
            </div>
            <div className="form-row">
              <ul className="project-entry-list">
                {Object.entries(existingProjects).map((project) => {
                  return (
                    <div
                      className="project-entry"
                      onClick={() => onClose(project[1])}
                    >
                      <label className="project-entry-name">{project[1]}</label>
                      <DeleteIcon
                        className="project-entry-delete icon"
                        title="Delete"
                        onClick={(e) => {
                          deleteProject(project[1]);
                          e.stopPropagation();
                        }}
                        fill={"var(--icon)"}
                      />
                    </div>
                  );
                })}
              </ul>
            </div>
            <div className="form-row">
              <div className="project-modal-creation-buttons-container">
                <div
                  className="project-modal-create-button"
                  onClick={() => {
                    setCreateProjectOpen(true);
                  }}
                >
                  Create New Project
                </div>
                {/* <div className='project-modal-create-button'>Other</div>
                <div className='project-modal-create-button'>Other</div> */}
              </div>
            </div>
          </>
        ) : (
          <>
            <div className="modal-titlebar">
              <BackIcon
                className="modal-titlebar-back icon"
                onClick={() => {
                  setCreateProjectOpen(false);
                }}
                fill={"var(--icon)"}
              />
              <label
                className="modal-titlebar-title"
                htmlFor="actionName"
                style={{ textAlign: "center" }}
              >
                Create New Project
              </label>
              <CloseIcon
                className="modal-titlebar-close icon"
                onClick={() => {
                  handleCancel();
                }}
                fill={"var(--icon)"}
              />
            </div>
            <div className="modal-complex-input-row-container">
              <div className="project-create-name modal-complex-input-container">
                <input
                  ref={focusInputRef}
                  type="text"
                  id="projectName"
                  name="projectName"
                  className="modal-complex-input"
                  onChange={handleInputChange}
                  autoComplete="off"
                  placeholder="Project Name"
                />
                <label for="projectName" class="modal-complex-input-label">
                  Project Name
                </label>
                <label
                  for="projectName"
                  class="modal-complex-input-indications"
                >
                  A unique name that is used for the project folder and other
                  resources. The name should be in lower case without spaces and
                  should not start with a number.
                </label>
              </div>
            </div>
            <div className="modal-complex-input-row-container">
              <div id="create-new-project" onClick={() => handleCreate()}>
                Create Project
              </div>
            </div>
          </>
        )}
      </form>
    </Modal>
  );
};

export default ProjectModal;
