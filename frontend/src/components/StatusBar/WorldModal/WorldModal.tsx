import React, { useState, useEffect } from "react";
import {
  Modal,
  ModalEditableList,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import CreatePage from "./CreatePage";
import { deleteWorld, listWorlds } from "BtApi/TreeWrapper";
import { useError } from "jderobot-ide-interface";
import CreateCustomPage from "./CreateCustomPage";
import ImportCustomPage from "./ImportCustomPage";
import "./WorldModal.css";

enum WorldTypes {
  ROBOTICSBACKEND,
  CUSTOM,
  ZIP,
}

const WorldModal = ({
  isOpen,
  onSelect,
  onClose,
  project,
}: {
  isOpen: boolean;
  onSelect: (world: string) => void;
  onClose: Function;
  project: string;
}) => {
  const { error } = useError();

  const [existingWorlds, setWorldsProjects] = useState([]);
  const [worldAdded, setWorldAdded] = useState(false);
  const [creationMenu, showCreationMenu] = useState<boolean>(false);
  const [creationType, changeCreationType] = useState<WorldTypes>(
    WorldTypes.CUSTOM,
  );

  const loadWorldsList = async () => {
    try {
      const response = await listWorlds(project);
      setWorldsProjects(response);
      setWorldAdded(false);
    } catch (e: unknown) {
      if (e instanceof Error) {
        error(e.message);
      }
    }
  };

  useEffect(() => {
    if (!isOpen || creationMenu) {
      return;
    }

    loadWorldsList();
    showCreationMenu(false);
  }, [isOpen, worldAdded, creationMenu]);

  const handleCancel = () => {
    if (project !== "") {
      onClose();
    }
  };

  const deleteWorldFunc = async (world: string) => {
    try {
      await deleteWorld(project, world);
      loadWorldsList();
      console.log("World deleted successfully");
    } catch (e: unknown) {
      if (e instanceof Error) {
        error(e.message);
      }
    }
  };

  const importFromRoboticsBackend = () => {
    showCreationMenu(true);
    changeCreationType(WorldTypes.ROBOTICSBACKEND);
  };

  const createCustomWorld = () => {
    showCreationMenu(true);
    changeCreationType(WorldTypes.CUSTOM);
  };

  const importFromZip = () => {
    showCreationMenu(true);
    changeCreationType(WorldTypes.ZIP);
  };

  return (
    <Modal
      id="worlds-modal"
      isOpen={isOpen}
      onClose={onClose}
      onReset={handleCancel}
    >
      {!creationMenu ? (
        <>
          <ModalTitlebar
            title="Manage your Worlds"
            htmlFor="actionName"
            hasClose
            handleClose={() => {
              handleCancel();
            }}
          />
          <ModalRow type="all">
            <ModalEditableList
              list={Object.values(existingWorlds)}
              onSelect={(e: any, entry: string) => {
                onSelect(entry);
              }}
              onDelete={(e: any, entry: string) => {
                deleteWorldFunc(entry);
                e.stopPropagation();
              }}
            />
          </ModalRow>
          <ModalRow type="buttons" id="worlds-buttons">
            <button
              type="button"
              onClick={createCustomWorld}
              style={{ width: "180px", height: "3em" }}
            >
              New custom world
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
          {creationType === WorldTypes.ROBOTICSBACKEND && (
            <CreatePage
              setVisible={showCreationMenu}
              visible={creationMenu}
              onClose={onClose}
              currentProject={project}
            />
          )}
          {creationType === WorldTypes.CUSTOM && (
            <CreateCustomPage
              setVisible={showCreationMenu}
              visible={creationMenu}
              onClose={onClose}
              currentProject={project}
            />
          )}
          {creationType === WorldTypes.ZIP && (
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

export default WorldModal;
