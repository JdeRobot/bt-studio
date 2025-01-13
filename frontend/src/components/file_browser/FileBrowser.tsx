import React, { useEffect, useState } from "react";
import JSZip from "jszip";
import "./FileBrowser.css";
import NewFileModal from "./modals/NewFileModal";
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

const FileBrowser = ({
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
  forceUpdate: { value: boolean, callback: Function};
  setSaveCurrentDiagram: Function;
}) => {
  const { warning, error } = useError();

  const [fileList, setFileList] = useState<Entry[]>([]);
  const [isNewFileModalOpen, setNewFileModalOpen] = useState(false);
  const [isNewFolderModalOpen, setNewFolderModalOpen] = useState(false);
  const [isRenameModalOpen, setRenameModalOpen] = useState(false);
  const [isDeleteModalOpen, setDeleteModalOpen] = useState(false);
  const [isUploadModalOpen, setUploadModalOpen] = useState(false);
  const [selectedEntry, setSelectedEntry] = useState<Entry|null>(null);
  const [deleteEntry, setDeleteEntry] = useState<string|null>(null);
  const [deleteType, setDeleteType] = useState(false);
  const [renameEntry, setRenameEntry] = useState<Entry|null>(null);
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

  const handleCloseNewFileModal = () => {
    setNewFileModalOpen(false);
    const file = document.getElementById("fileName");

    if (file) {
      (file as HTMLFormElement).value = "";
    }
  };

  const handleNewActionSubmit = async (location:string, data:any) => {
    handleCloseNewFileModal();

    if (data.fileName !== "") {
      try {
        switch (data.fileType) {
          case "actions":
            await createAction(
              currentProjectname,
              data.fileName,
              data.templateType,
            );
            break;
          default:
            await createFile(currentProjectname, data.fileName, location);
            break;
        }
        setProjectChanges(true);
        fetchFileList(); // Update the file list
      } catch (e) {
        if (e instanceof Error) {
          console.error("Error creating file:", e);
          error("Error creating file" + e.message);
        }
      }
    }
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

  const handleCloseDeleteModal = () => {
    setDeleteModalOpen(false);
    setDeleteEntry("");
    setDeleteType(false);
  };

  const handleSubmitDeleteModal = async () => {
    //currentFilename === Absolute File path
    if (deleteEntry) {
      try {
        if (deleteType) {
          await deleteFolder(currentProjectname, deleteEntry);
        } else {
          await deleteFile(currentProjectname, deleteEntry);
        }

        setProjectChanges(true);
        fetchFileList(); // Update the file list

        if (currentFilename === deleteEntry) {
          setCurrentFilename(""); // Unset the current file
        }
        if (selectedEntry && selectedEntry.path === deleteEntry) {
          setSelectedEntry(null);
        }
      } catch (e) {
        if (e instanceof Error) {
          console.error("Error deleting file:", e);
          error("Error deleting file" + e.message);
        }
      }
    } else {
      warning("No file is currently selected.");
    }
    handleCloseDeleteModal();
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

  const handleCloseCreateFolder = () => {
    setNewFolderModalOpen(false);
    const folder = document.getElementById("folderName");

    if (folder) {
      (folder as HTMLFormElement).value = "";
    }
  };

  const handleCreateFolderSubmit = async (location: string, folder_name:string) => {
    if (folder_name !== "") {
      try {
        await createFolder(currentProjectname, folder_name, location);
        setProjectChanges(true);
        fetchFileList(); // Update the file list
      } catch (e) {
        if (e instanceof Error) {
          console.error("Error creating folder:", e.message);
          error("Error creating folder: " + e.message);
        }
      }
    }
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

  const handleCloseRenameModal = () => {
    setRenameModalOpen(false);
  };

  const handleSubmitRenameModal = async (new_path: string) => {
    if (renameEntry) {
      try {
        console.log(renameEntry);
        if (renameEntry.is_dir) {
          await renameFolder(currentProjectname, renameEntry.path, new_path);
        } else {
          await renameFile(currentProjectname, renameEntry.path, new_path);
        }

        setProjectChanges(true);
        fetchFileList(); // Update the file list

        if (currentFilename === renameEntry.path) {
          setAutosave(false);
          setCurrentFilename(new_path); // Unset the current file
        }
      } catch (e) {
        if (e instanceof Error) {
          console.error("Error deleting file:", e);
          error("Error deleting file: " + e.message);
        }
      }
    } else {
      warning("No file is currently selected.");
    }
    handleCloseRenameModal();
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

  ///////////////// UPLOAD /////////////////////////////////////////////////////

  const handleUpload = (file: Entry) => {
    updateSelectedLocation(file);
    setUploadModalOpen(true);
    setSaveCurrentDiagram(true);
  };

  const handleCloseUploadModal = () => {
    setUploadModalOpen(false);
    fetchFileList();
  };

  ///////////////// DOWNLOAD ///////////////////////////////////////////////////
  const zipFile = async (zip: JSZip, file_path: string, file_name: string) => {
    var content = await getFile(currentProjectname, file_path);
    zip.file(file_name, content);
  };

  const zipFolder = async (zip: JSZip, file:Entry) => {
    const folder = zip.folder(file.name);

    if (folder === null) {
      return
    }

    for (let index = 0; index < file.files.length; index++) {
      const element = file.files[index];
      console.log(element);
      if (element.is_dir) {
        await zipFolder(folder, element);
      } else {
        await zipFile(folder, element.path, element.name);
      }
    }
  };

  const handleDownload = async (file: Entry) => {
    if (file) {
      try {
        // Create the zip with the files
        const zip = new JSZip();

        if (file.is_dir) {
          await zipFolder(zip, file);
        } else {
          await zipFile(zip, file.path, file.name);
        }

        zip.generateAsync({ type: "blob" }).then(function (content) {
          // Create a download link and trigger download
          const url = window.URL.createObjectURL(content);
          const a = document.createElement("a");
          a.style.display = "none";
          a.href = url;
          a.download = `${file.name.split(".")[0]}.zip`; // Set the downloaded file's name
          document.body.appendChild(a);
          a.click();
          window.URL.revokeObjectURL(url); // Clean up after the download
        });
      } catch (e) {
        if (e instanceof Error) {
          console.error("Error downloading file: " + e);
          error("Error downloading file: " + e.message);
        }
      }
    }
  };

  return (
    <div className="bt-sidebar-content">
      <div className="bt-sidebar-entry">
        <div className="bt-sidebar-entry-menu">
          <button
            className="bt-sidebar-button"
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
          <div style={{ marginLeft: "auto" }} />
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
        <FileExplorer
          setCurrentFilename={setCurrentFilename}
          currentFilename={currentFilename}
          currentProjectname={currentProjectname}
          setSelectedEntry={setSelectedEntry}
          fileList={fileList}
          fetchFileList={fetchFileList}
          onDelete={handleDeleteModal}
          onCreateFile={handleCreateFile}
          onCreateFolder={handleCreateFolder}
          onUpload={handleUpload}
          onDownload={handleDownload}
          onRename={handleRename}
        />
      </div>
      <NewFileModal
        isOpen={isNewFileModalOpen}
        onSubmit={handleNewActionSubmit}
        onClose={handleCloseNewFileModal}
        fileList={fileList}
        location={selectedLocation}
      />
      <NewFolderModal
        isOpen={isNewFolderModalOpen}
        onSubmit={handleCreateFolderSubmit}
        onClose={handleCloseCreateFolder}
        fileList={fileList}
        location={selectedLocation}
      />
      <RenameModal
        isOpen={isRenameModalOpen}
        onSubmit={handleSubmitRenameModal}
        onClose={handleCloseRenameModal}
        fileList={fileList}
        selectedEntry={renameEntry}
      />
      <UploadModal
        isOpen={isUploadModalOpen}
        onSubmit={handleCloseUploadModal}
        onClose={handleCloseUploadModal}
        location={selectedLocation}
        currentProject={currentProjectname}
      />
      <DeleteModal
        isOpen={isDeleteModalOpen}
        onSubmit={handleSubmitDeleteModal}
        onClose={handleCloseDeleteModal}
        selectedEntry={deleteEntry}
      />
    </div>
  );
};

export default FileBrowser;
