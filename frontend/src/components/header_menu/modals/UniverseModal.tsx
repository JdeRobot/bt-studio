import React, { useState, useEffect, useRef, FormEventHandler } from "react";
import "./UniverseModal.css";
import Modal from "../../Modal/Modal";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";
import { ReactComponent as DeleteIcon } from "../../tree_editor/img/delete.svg";
import CreatePage from "./universe/CreatePage";
import axios from "axios";
import UniverseUploadModal from "./UniverseUploadModal";

const UniverseModal = ({
  onSubmit,
  isOpen,
  onClose,
  currentProject,
  openError,
}: {
  onSubmit: FormEventHandler<HTMLFormElement>;
  isOpen: boolean;
  onClose: Function;
  currentProject: string;
  openError: Function;
}) => {
  const focusInputRef = useRef<any>(null);
  const [existingUniverses, setUniversesProjects] = useState([]);
  const [uploadModalOpen, setUploadModalOpen] = useState(false);
  const [universeAdded, setUniverseAdded] = useState(false);
  const [creationMenu, showCreationMenu] = useState<boolean>(false);

  const loadUniverseList = async () => {
    try {
      const listApiUrl = `/bt_studio/get_universes_list?project_name=${currentProject}`;
      const response = await axios.get(listApiUrl);
      setUniversesProjects(response.data.universes_list);
      setUniverseAdded(false);
    } catch (error) {
      console.error("Error while fetching universes list:", error);
      openError(`An error occurred while fetching the universes list`);
    }
  };

  useEffect(() => {
    if (!isOpen || creationMenu) {
      return;
    }

    if (focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    loadUniverseList();
    showCreationMenu(false);
  }, [isOpen, universeAdded, creationMenu]);

  const handleCancel = () => {
    if (currentProject !== "") {
      onClose();
    }
  };

  const deleteUniverse = async (universe_name: string) => {
    try {
      const apiUrl = `/bt_studio/delete_universe?project_name=${currentProject}&universe_name=${universe_name}`;
      const response = await axios.get(apiUrl);
      if (response.data.success) {
        loadUniverseList();
        console.log("Universe deleted successfully");
      }
    } catch (error: any) {
      if (error.response) {
        // The request was made and the server responded with a status code
        // that falls out of the range of 2xx
        if (error.response.status === 409) {
          openError(`The universe ${universe_name} does not exist`);
        } else {
          // Handle other statuses or general API errors
          openError(
            "Unable to connect with the backend server. Please check the backend status.",
          );
        }
      }
    }
  };

  const importFromRoboticsBackend = () => {
    console.log("Create from RB");
    //TODO: need to get the name and open a dropdown to select the universe
    showCreationMenu(true);
  };

  const importFromZip = () => {
    setUploadModalOpen(true);
  };

  const handleCloseUploadUniverseModal = (universe_name: string) => {
    setUploadModalOpen(false);
  };

  const handleFormSubmit = (data: any) => {};

  return (
    <Modal
      id="universes-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <UniverseUploadModal
        isOpen={uploadModalOpen}
        onSubmit={handleFormSubmit}
        onClose={handleCloseUploadUniverseModal}
        currentProject={currentProject}
        openError={openError}
        setUniverseAdded={setUniverseAdded}
      />
      <form onSubmit={onSubmit} onReset={handleCancel}>
        {!creationMenu ? (
          <>
            <div className="modal-titlebar">
              <label
                className="modal-titlebar-title"
                htmlFor="actionName"
                style={{ textAlign: "center" }}
              >
                Manage your Universes
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
                {Object.entries(existingUniverses).map((project) => {
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
                          deleteUniverse(project[1]);
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
                    importFromZip();
                  }}
                >
                  Import from zip
                </div>
                <div
                  className="project-modal-create-button"
                  onClick={() => {
                    importFromRoboticsBackend();
                  }}
                >
                  Import from Robotics Backend library
                </div>
                {/* <div className='project-modal-create-button'>Other</div> */}
              </div>
            </div>
          </>
        ) : (
          <>
            <CreatePage
              setVisible={showCreationMenu}
              visible={creationMenu}
              onClose={onClose}
              currentProject={currentProject}
              openError={openError}
            />
          </>
        )}
      </form>
    </Modal>
  );
};

export default UniverseModal;
