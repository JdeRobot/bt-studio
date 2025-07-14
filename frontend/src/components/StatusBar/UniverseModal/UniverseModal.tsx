import React, { useState, useEffect, useRef } from "react";
import {
  Modal,
  ModalEditableList,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import CreatePage from "./CreatePage";
import { deleteUniverse, listUniverses } from "../../../api_helper/TreeWrapper";
import { useError } from "jderobot-ide-interface";
import CreateCustomPage from "./CreateCustomPage";
import ImportCustomPage from "./ImportCustomPage";
import "./UniverseModal.css";

enum UniverseTypes {
  ROBOTICSBACKEND,
  CUSTOM,
  ZIP,
}

const UniverseModal = ({
  isOpen,
  onSelect,
  onClose,
  project,
}: {
  isOpen: boolean;
  onSelect: (universe: string) => void;
  onClose: Function;
  project: string;
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
      const response = await listUniverses(project);
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
    if (project !== "") {
      onClose();
    }
  };

  const deleteUniverseFunc = async (universe_name: string) => {
    try {
      await deleteUniverse(project, universe_name);
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
      isOpen={isOpen}
      onClose={onClose}
      onSubmit={(data: unknown) => {}}
      onReset={handleCancel}
    >
      {!creationMenu ? (
        <>
          <ModalTitlebar
            title="Manage your Universes"
            htmlFor="actionName"
            hasClose
            handleClose={() => {
              handleCancel();
            }}
          />
          <ModalRow type="all">
            <ModalEditableList
              list={Object.values(existingUniverses)}
              onSelect={(e: any, entry: string) => {
                onSelect(entry);
              }}
              onDelete={(e: any, entry: string) => {
                deleteUniverseFunc(entry);
                e.stopPropagation();
              }}
            />
          </ModalRow>
          <ModalRow type="buttons" id="universe-buttons">
            <button
              type="button"
              onClick={createCustomUniverse}
              style={{ width: "180px", height: "3em" }}
            >
              New custom universe
            </button>
            <button
              type="button"
              onClick={importFromZip}
              style={{ width: "180px", height: "3em" }}
            >
              Import from zip
            </button>
            <button
              type="button"
              onClick={importFromRoboticsBackend}
              style={{ width: "180px", height: "3em" }}
            >
              Import from Robotics Backend library
            </button>
          </ModalRow>
        </>
      ) : (
        <>
          {creationType === UniverseTypes.ROBOTICSBACKEND && (
            <CreatePage
              setVisible={showCreationMenu}
              visible={creationMenu}
              onClose={onClose}
              currentProject={project}
            />
          )}
          {creationType === UniverseTypes.CUSTOM && (
            <CreateCustomPage
              setVisible={showCreationMenu}
              visible={creationMenu}
              onClose={onClose}
              currentProject={project}
            />
          )}
          {creationType === UniverseTypes.ZIP && (
            <ImportCustomPage
              setVisible={showCreationMenu}
              visible={creationMenu}
              onClose={onClose}
              currentProject={project}
            />
          )}
        </>
      )}
    </Modal>
  );
};

export default UniverseModal;
