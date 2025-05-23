import React, { useState, useEffect, useRef, FormEventHandler } from "react";
import "./UniverseModal.css";
import Modal from "../../Modal/Modal";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";
import { ReactComponent as DeleteIcon } from "../../tree_editor/img/delete.svg";
import CreatePage from "./universe/CreatePage";
import { deleteUniverse, listUniverses } from "../../../api_helper/TreeWrapper";
import { useError } from "../../error_popup/ErrorModal";
import CreateCustomPage from "./universe/CreateCustomPage";
import ImportCustomPage from "./universe/ImportCustomPage";

enum UniverseTypes {
  ROBOTICSBACKEND,
  CUSTOM,
  ZIP,
}

const UniverseModal = ({
  onSubmit,
  isOpen,
  onClose,
  currentProject,
}: {
  onSubmit: FormEventHandler<HTMLFormElement>;
  isOpen: boolean;
  onClose: Function;
  currentProject: string;
}) => {
  const { error } = useError();

  const focusInputRef = useRef<any>(null);
  const [existingUniverses, setUniversesProjects] = useState([]);
  const [uploadModalOpen, setUploadModalOpen] = useState(false);
  const [universeAdded, setUniverseAdded] = useState(false);
  const [creationMenu, showCreationMenu] = useState<boolean>(false);
  const [creationType, changeCreationType] = useState<UniverseTypes>(
    UniverseTypes.CUSTOM,
  );

  const loadUniverseList = async () => {
    try {
      const response = await listUniverses(currentProject);
      setUniversesProjects(response);
      setUniverseAdded(false);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching universes list: " + e.message);
        error("Error while fetching universes list: " + e.message);
      }
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

  const deleteUniverseFunc = async (universe_name: string) => {
    try {
      await deleteUniverse(currentProject, universe_name);
      loadUniverseList();
      console.log("Universe deleted successfully");
    } catch (e: any) {
      if (e.response) {
        // The request was made and the server responded with a status code
        // that falls out of the range of 2xx
        if (e.response.status === 409) {
          error(`The universe ${universe_name} does not exist`);
        } else {
          // Handle other statuses or general API errors
          error(
            "Unable to connect with the backend server. Please check the backend status.",
          );
        }
      }
    }
  };

  const importFromRoboticsBackend = () => {
    showCreationMenu(true);
    changeCreationType(UniverseTypes.ROBOTICSBACKEND);
  };

  const createCustomUniverse = () => {
    showCreationMenu(true);
    changeCreationType(UniverseTypes.CUSTOM);
  };

  const importFromZip = () => {
    showCreationMenu(true);
    changeCreationType(UniverseTypes.ZIP);
  };

  const handleCloseUploadUniverseModal = (universe_name: string) => {
    setUploadModalOpen(false);
  };

  return (
    <Modal
      id="universes-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={onSubmit} onReset={handleCancel}>
        {!creationMenu ? (
          <>
            <div className="bt-modal-titlebar">
              <label
                className="bt-modal-titlebar-title"
                htmlFor="actionName"
                style={{ textAlign: "center" }}
              >
                Manage your Universes
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
                {Object.entries(existingUniverses).map((project) => {
                  return (
                    <div
                      className="bt-project-entry"
                      id={"project-" + project[1]}
                      onClick={() => onClose(project[1])}
                    >
                      <label className="bt-project-entry-name">
                        {project[1]}
                      </label>
                      <DeleteIcon
                        className="bt-project-entry-delete bt-icon"
                        title="Delete"
                        id={"delete-project-" + project[1]}
                        onClick={(e) => {
                          deleteUniverseFunc(project[1]);
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
                  onClick={() => {
                    createCustomUniverse();
                  }}
                >
                  New custom universe
                </div>
                <div
                  className="bt-project-modal-create-button"
                  onClick={() => {
                    importFromZip();
                  }}
                >
                  Import from zip
                </div>
                <div
                  className="bt-project-modal-create-button"
                  id="open-import-rb-universe"
                  onClick={() => {
                    importFromRoboticsBackend();
                  }}
                >
                  Import from Robotics Backend library
                </div>
              </div>
            </div>
          </>
        ) : (
          <>
            {creationType === UniverseTypes.ROBOTICSBACKEND && (
              <CreatePage
                setVisible={showCreationMenu}
                visible={creationMenu}
                onClose={onClose}
                currentProject={currentProject}
              />
            )}
            {creationType === UniverseTypes.CUSTOM && (
              <CreateCustomPage
                setVisible={showCreationMenu}
                visible={creationMenu}
                onClose={onClose}
                currentProject={currentProject}
              />
            )}
            {creationType === UniverseTypes.ZIP && (
              <ImportCustomPage
                setVisible={showCreationMenu}
                visible={creationMenu}
                onClose={onClose}
                currentProject={currentProject}
              />
            )}
          </>
        )}
      </form>
    </Modal>
  );
};

export default UniverseModal;
