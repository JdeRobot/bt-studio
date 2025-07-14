import React, { useState, useEffect, useRef, FormEventHandler } from "react";
import "./ProjectModal.css";
import { deleteProject, listProjects } from "../../../api_helper/TreeWrapper";
import {
  Modal,
  ModalEditableList,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
  useError,
} from "jderobot-ide-interface";

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

  const handleCreate = async () => {
    if (formState.projectName === "") {
      return;
    }
    setCreateProjectOpen(false);
    await createProject(formState.projectName);
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
      isOpen={isOpen}
      onClose={onClose}
      onSubmit={onSubmit}
      onReset={handleCancel}
    >
      {!createProjectOpen ? (
        <>
          <ModalTitlebar
            title="Open a Project"
            htmlFor="actionName"
            hasClose
            handleClose={() => {
              handleCancel();
            }}
          />
          <ModalRow type="all">
            <ModalEditableList
              list={Object.values(existingProjects)}
              onSelect={(e: any, entry: string) => {
                onClose(entry);
              }}
              onDelete={(e: any, entry: string) => {
                deleteProjectFunc(entry);
                e.stopPropagation();
              }}
            />
          </ModalRow>
          <ModalRow type="buttons" id="project-buttons">
            <button
              type="button"
              onClick={() => {
                setCreateProjectOpen(true);
              }}
              style={{ width: "180px", height: "3em" }}
            >
              Create New Project
            </button>
            {/* <div className='bt-project-modal-create-button'>Other</div>
              <div className='bt-project-modal-create-button'>Other</div> */}
          </ModalRow>
        </>
      ) : (
        <>
          <ModalTitlebar
            title="Create New Project"
            htmlFor="actionName"
            hasClose
            hasBack
            handleClose={() => {
              handleCancel();
            }}
            handleBack={() => {
              setCreateProjectOpen(false);
            }}
          />
          <ModalRow type="input">
            <ModalInputBox
              isInputValid={true}
              ref={focusInputRef}
              id="projectName"
              placeholder="Project Name"
              onChange={handleInputChange}
              description="A unique name that is used for the project folder and other
                resources. The name should be in lower case without spaces and
                should not start with a number."
              type="text"
              autoComplete="off"
              required
            />
          </ModalRow>
          <ModalRow type="buttons" id="project-buttons">
            <button
              type="button"
              onClick={() => {
                handleCreate();
              }}
              style={{ width: "180px", height: "3em" }}
            >
              Create Project
            </button>
          </ModalRow>
        </>
      )}
    </Modal>
  );
};

export default ProjectModal;
