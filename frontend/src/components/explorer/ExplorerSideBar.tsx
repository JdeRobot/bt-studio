import React, { useEffect, useState } from "react";
import JSZip from "jszip";
import "./ExplorerSideBar.css";
import NewFileModal, { newFileModalData } from "./modals/NewFileModal";
import RenameModal from "./modals/RenameModal";
import NewFolderModal from "./modals/NewFolderModal";
import UploadModal from "./modals/UploadModal";
import DeleteModal from "./modals/DeleteModal";
import FileExplorer from "./file_explorer/FileExplorer";

import {
  getFile,
  getFileList,
  createAction,
  createFile,
  createFolder,
  renameFile,
  renameFolder,
  deleteFile,
  deleteFolder,
} from "../../api_helper/TreeWrapper";

import { ReactComponent as AddIcon } from "./img/add.svg";
import { ReactComponent as AddFolderIcon } from "./img/add_folder.svg";
import { ReactComponent as DeleteIcon } from "./img/delete.svg";
import { ReactComponent as RefreshIcon } from "./img/refresh.svg";
import { ReactComponent as RenameIcon } from "./img/rename.svg";

import { useError } from "../error_popup/ErrorModal";
import { publish } from "../helper/TreeEditorHelper";
import ExplorerContainer from "./ExplorerContainer";

export interface Entry {
  name: string;
  is_dir: boolean;
  path: string;
  files: Entry[];
}

function getParentDir(file: Entry) {
  // Check if is a directory and if not get the parent directory of the file
  if (file.is_dir) {
    return file.path;
  }

  var split_path = file.path.split("/");
  return split_path.slice(0, split_path.length - 1).join("/");
}

const ExplorerSideBar = ({
  setCurrentFilename,
  currentFilename,
  currentProjectname,
  setProjectChanges,
  setAutosave,
  forceSaveCurrent,
  setForcedSaveCurrent,
  forceUpdate,
  setSaveCurrentDiagram,
}: {
  setCurrentFilename: Function;
  currentFilename: string;
  currentProjectname: string;
  setProjectChanges: Function;
  setAutosave: Function;
  forceSaveCurrent: boolean;
  setForcedSaveCurrent: Function;
  forceUpdate: { value: boolean; callback: Function };
  setSaveCurrentDiagram: Function;
}) => {
  const { warning, error } = useError();

  const [fileList, setFileList] = useState<Entry[]>([]);
  const [isNewFileModalOpen, setNewFileModalOpen] = useState(false);
  const [isNewFolderModalOpen, setNewFolderModalOpen] = useState(false);
  const [isRenameModalOpen, setRenameModalOpen] = useState(false);
  const [isDeleteModalOpen, setDeleteModalOpen] = useState(false);
  const [isUploadModalOpen, setUploadModalOpen] = useState(false);
  const [selectedEntry, setSelectedEntry] = useState<Entry | null>(null);
  const [deleteEntry, setDeleteEntry] = useState<string | null>(null);
  const [deleteType, setDeleteType] = useState(false);
  const [renameEntry, setRenameEntry] = useState<Entry | null>(null);
  const [selectedLocation, setSelectedLocation] = useState("");

  useEffect(() => {
    updateSelectedLocation(undefined);
  }, [selectedEntry]);

  useEffect(() => {
    if (forceUpdate.value) {
      forceUpdate.callback(false);
      fetchFileList();
    }
  }, [forceUpdate.value]);

  const updateSelectedLocation = (file: Entry | undefined) => {
    if (file) {
      setSelectedLocation(getParentDir(file));
    } else {
      if (selectedEntry) {
        setSelectedLocation(getParentDir(selectedEntry));
      } else {
        setSelectedLocation("");
      }
    }
  };

  const fetchFileList = async () => {
    console.log("Fecthing file list, the project name is:", currentProjectname);
    if (currentProjectname !== "") {
      try {
        const file_list = await getFileList(currentProjectname);
        const files = JSON.parse(file_list);
        setFileList(files);
      } catch (e) {
        if (e instanceof Error) {
          console.error("Error fetching files:", e);
          error("Error fetching files: " + e.message);
        }
      }
    }
  };

  ///////////////// CREATE FILES ///////////////////////////////////////////////

  const handleCreateFile = (file: Entry | undefined) => {
    updateSelectedLocation(file);
    setNewFileModalOpen(true);
    setSaveCurrentDiagram(true);
  };

  ///////////////// DELETE FILES AND FOLDERS ///////////////////////////////////

  const handleDeleteModal = (file_path: string, is_dir: boolean) => {
    if (file_path) {
      setDeleteEntry(file_path);
      setDeleteType(is_dir);
      setDeleteModalOpen(true);
      setSaveCurrentDiagram(true);
    } else {
      warning("No file is currently selected.");
    }
  };


  const handleDeleteCurrentFile = () => {
    //currentFilename === Absolute File path
    if (currentFilename) {
      handleDeleteModal(currentFilename, false);
      setAutosave(false);
    } else {
      warning("No file is currently selected.");
    }
  };

  ///////////////// CREATE FOLDER //////////////////////////////////////////////

  const handleCreateFolder = (file: Entry | undefined) => {
    updateSelectedLocation(file);
    setNewFolderModalOpen(true);
    setSaveCurrentDiagram(true);
  };

  ///////////////// RENAME /////////////////////////////////////////////////////

  const handleRename = (file: Entry) => {
    if (file) {
      setRenameEntry(file);
      setRenameModalOpen(true);
      setSaveCurrentDiagram(true);
    } else {
      warning("No file is currently selected.");
    }

    if (currentFilename === file.path) {
      setForcedSaveCurrent(!forceSaveCurrent);
    }
  };

  const handleRenameCurrentFile = async () => {
    if (currentFilename) {
      setForcedSaveCurrent(!forceSaveCurrent);
      const name = currentFilename.substring(
        currentFilename.lastIndexOf("/") + 1,
      );
      handleRename({
        is_dir: false,
        name: name,
        path: currentFilename,
        files: [],
      });
      setAutosave(false);
    } else {
      warning("No file is currently selected.");
    }
  };

  return (
    <div className="bt-sidebar-content">
      <div className="bt-sidebar-entry">
        <div className="bt-sidebar-entry-menu">
          <div style={{ marginLeft: "auto" }} />
          <button
            className="bt-sidebar-button"
            id="create-file-open"
            onClick={() => handleCreateFile(undefined)}
            title="Create a new file"
          >
            <AddIcon className="bt-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-sidebar-button"
            onClick={() => handleCreateFolder(undefined)}
            title="Create a new folder"
          >
            <AddFolderIcon className="bt-icon" stroke={"var(--icon)"} />
          </button>
          <button
            className="bt-sidebar-button"
            onClick={() => fetchFileList()}
            title="Refresh View"
          >
            <RefreshIcon className="bt-icon" stroke={"var(--icon)"} />
          </button>
          {currentFilename !== "" && (
            <>
              <button
                className="bt-sidebar-button"
                onClick={handleRenameCurrentFile}
                title="Rename file"
              >
                <RenameIcon className="bt-icon" stroke={"var(--icon)"} />
              </button>
              <button
                className="bt-sidebar-button"
                onClick={handleDeleteCurrentFile}
                title="Delete file"
              >
                <DeleteIcon className="bt-icon" fill={"var(--icon)"} />
              </button>
            </>
          )}
        </div>
        {/* <ExplorerContainer>
        </ExplorerContainer> */}
      </div>
    </div>
  );
};

export default ExplorerSideBar;
