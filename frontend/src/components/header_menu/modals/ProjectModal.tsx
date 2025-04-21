import React, { useState, useEffect, useRef, FormEventHandler } from "react";
import "./ProjectModal.css";
import Modal from "../../Modal/Modal";
import { ReactComponent as BackIcon } from "../../Modal/img/back.svg";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";
import { ReactComponent as DeleteIcon } from "../../tree_editor/img/delete.svg";
import { deleteProject, listProjects } from "../../../api_helper/TreeWrapper";
import { useError } from "../../error_popup/ErrorModal";

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
}: {
  onSubmit: FormEventHandler<HTMLFormElement>;
  isOpen: boolean;
  onClose: Function;
  currentProject: string;
  existingProjects: string[];
  setExistingProjects: Function;
  createProject: Function;
}) => {
  const { error } = useError();

  const focusInputRef = useRef<any>(null);
  const [createProjectOpen, setCreateProjectOpen] = useState(false);
  const [formState, setFormState] = useState(initialProjectData);

  const getProjects = async () => {
    try {
      const response = await listProjects();
      setExistingProjects(response);
      setFormState(initialProjectData);
    } catch (e) {
      setExistingProjects([]);
      setFormState(initialProjectData);
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        // error("Error while fetching project list: " + e.message);
      }
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

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

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

  const deleteProjectFunc = async (project: string) => {
    if (currentProject === project) {
      //TODO: change this to change project before deleting
      return;
    }

    // Delete and update
    try {
      await deleteProject(project);
      await getProjects();
      console.log("Project deleted successfully");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        error("Error while fetching project list: " + e.message);
      }
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
            <div className="bt-modal-titlebar">
              <label
                className="bt-modal-titlebar-title"
                htmlFor="actionName"
                style={{ textAlign: "center" }}
              >
                Open a Project
              </label>
              <CloseIcon
                className="bt-modal-titlebar-close bt-icon"
                onClick={() => {
                  handleCancel();
                }}
                fill={"var(--icon)"}
              />
            </div>
            <div className="bt-form-row">
              <ul className="bt-project-entry-list">
                {Object.entries(existingProjects).map((project) => {
                  return (
                    <div
                      className="bt-project-entry"
                      onClick={() => onClose(project[1])}
                    >
                      <label className="bt-project-entry-name">
                        {project[1]}
                      </label>
                      <DeleteIcon
                        className="bt-project-entry-delete bt-icon"
                        title="Delete"
                        onClick={(e) => {
                          deleteProjectFunc(project[1]);
                          e.stopPropagation();
                        }}
                        fill={"var(--icon)"}
                      />
                    </div>
                  );
                })}
              </ul>
            </div>
            <div className="bt-form-row">
              <div className="bt-project-modal-creation-buttons-container">
                <div
                  className="bt-project-modal-create-button"
                  id="create-project-open"
                  onClick={() => {
                    setCreateProjectOpen(true);
                  }}
                >
                  Create New Project
                </div>
                {/* <div className='bt-project-modal-create-button'>Other</div>
                <div className='bt-project-modal-create-button'>Other</div> */}
              </div>
            </div>
          </>
        ) : (
          <>
            <div className="bt-modal-titlebar">
              <BackIcon
                className="bt-modal-titlebar-back bt-icon"
                onClick={() => {
                  setCreateProjectOpen(false);
                }}
                fill={"var(--icon)"}
              />
              <label
                className="bt-modal-titlebar-title"
                htmlFor="actionName"
                style={{ textAlign: "center" }}
              >
                Create New Project
              </label>
              <CloseIcon
                className="bt-modal-titlebar-close bt-icon"
                onClick={() => {
                  handleCancel();
                }}
                fill={"var(--icon)"}
              />
            </div>
            <div className="bt-modal-complex-input-row-container">
              <div className="bt-project-create-name bt-modal-complex-input-container">
                <input
                  ref={focusInputRef}
                  type="text"
                  id="projectName"
                  name="projectName"
                  className="bt-modal-complex-input"
                  onChange={handleInputChange}
                  autoComplete="off"
                  placeholder="Project Name"
                />
                <label
                  htmlFor="projectName"
                  className="bt-modal-complex-input-label"
                >
                  Project Name
                </label>
                <label
                  htmlFor="projectName"
                  className="bt-modal-complex-input-indications"
                >
                  A unique name that is used for the project folder and other
                  resources. The name should be in lower case without spaces and
                  should not start with a number.
                </label>
              </div>
            </div>
            <div className="bt-modal-complex-input-row-container">
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
