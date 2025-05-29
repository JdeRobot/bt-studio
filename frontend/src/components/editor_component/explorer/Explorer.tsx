import React, { useEffect, useState } from "react";
import JSZip from "jszip";
import "./Explorer.css";
import NewFileModal, { newFileModalData } from "./modals/NewFileModal";
import RenameModal from "./modals/RenameModal";
import NewFolderModal from "./modals/NewFolderModal";
import UploadModal from "./modals/UploadModal";
import DeleteModal from "./modals/DeleteModal";
import FileExplorer from "./file_explorer/FileExplorer";

import { ReactComponent as AddIcon } from "./img/add.svg";
import { ReactComponent as AddFolderIcon } from "./img/add_folder.svg";
import { ReactComponent as DeleteIcon } from "./img/delete.svg";
import { ReactComponent as RefreshIcon } from "./img/refresh.svg";
import { ReactComponent as RenameIcon } from "./img/rename.svg";

import { useError } from "../../error_popup/ErrorModal";

export interface ExplorerEntry {
  name: string;
  list(project: string): Promise<string>;
  file: {
    create(
      project: string,
      location: string,
      data: newFileModalData,
    ): Promise<void>;
    get(project: string, path: string): Promise<string>;
    rename(project: string, oldPath: string, newPath: string): Promise<void>;
    delete(project: string, path: string): Promise<void>;
  };
  folder: {
    create(project: string, location: string, name: string): Promise<void>;
    rename(project: string, oldPath: string, newPath: string): Promise<void>;
    delete(project: string, path: string): Promise<void>;
  };
}

export interface Entry {
  name: string;
  is_dir: boolean;
  path: string;
  group: string;
  access: boolean;
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

const Explorer = ({
  setCurrentFile,
  currentFile,
  project,
  api,
}: {
  setCurrentFile: Function;
  currentFile?: Entry;
  project: string;
  api: ExplorerEntry;
}) => {
  const { warning, error } = useError();

  const [fileList, setFileList] = useState<Entry[]>([]);
  const [deleteEntry, setDeleteEntry] = useState<Entry | undefined>(undefined);
  const [renameEntry, setRenameEntry] = useState<Entry | undefined>(undefined);
  const [selectedEntry, setSelectedEntry] = useState<Entry | undefined>(
    undefined,
  );
  const [selectedLocation, setSelectedLocation] = useState("");

  const [isNewFileModalOpen, setNewFileModalOpen] = useState(false);
  const [isNewFolderModalOpen, setNewFolderModalOpen] = useState(false);
  const [isRenameModalOpen, setRenameModalOpen] = useState(false);
  const [isDeleteModalOpen, setDeleteModalOpen] = useState(false);
  const [isUploadModalOpen, setUploadModalOpen] = useState(false);
  const [deleteType, setDeleteType] = useState(false);

  useEffect(() => {
    updateSelectedLocation(undefined);
  }, [selectedEntry]);

  // useEffect(() => {
  //   if (forceUpdate.value) {
  //     forceUpdate.callback(false);
  //     fetchFileList();
  //   }
  // }, [forceUpdate.value]);

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
    console.log("Fecthing file list, the project name is:", project);
    if (project !== "") {
      try {
        const file_list = await api.list(project);
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
  };

  const handleCloseNewFileModal = () => {
    setNewFileModalOpen(false);
    const file = document.getElementById("fileName");

    if (file) {
      (file as HTMLFormElement).value = "";
    }
  };

  const handleNewActionSubmit = async (
    location: string,
    data: newFileModalData,
  ) => {
    handleCloseNewFileModal();

    if (data.fileName !== "") {
      try {
        await api.file.create(project, location, data);
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

  const handleDeleteModal = (file: Entry, is_dir: boolean) => {
    if (file) {
      setDeleteEntry(file);
      setDeleteType(is_dir);
      setDeleteModalOpen(true);
    } else {
      warning("No file is currently selected.");
    }
  };

  const handleCloseDeleteModal = () => {
    setDeleteModalOpen(false);
    setDeleteEntry(undefined);
    setDeleteType(false);
  };

  const handleSubmitDeleteModal = async () => {
    //currentFile === Absolute File path
    if (deleteEntry) {
      try {
        if (deleteType) {
          await api.folder.delete(project, deleteEntry.path);
        } else {
          await api.file.delete(project, deleteEntry.path);
        }

        fetchFileList(); // Update the file list

        if (currentFile === deleteEntry) {
          setCurrentFile(undefined); // Unset the current file
        }
        if (selectedEntry && selectedEntry.path === deleteEntry.path) {
          setSelectedEntry(undefined);
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
    //currentFile === Absolute File path
    if (currentFile) {
      handleDeleteModal(currentFile, false);
    } else {
      warning("No file is currently selected.");
    }
  };

  ///////////////// CREATE FOLDER //////////////////////////////////////////////

  const handleCreateFolder = (file: Entry | undefined) => {
    updateSelectedLocation(file);
    setNewFolderModalOpen(true);
  };

  const handleCloseCreateFolder = () => {
    setNewFolderModalOpen(false);
    const folder = document.getElementById("folderName");

    if (folder) {
      (folder as HTMLFormElement).value = "";
    }
  };

  const handleCreateFolderSubmit = async (
    location: string,
    folder_name: string,
  ) => {
    if (folder_name !== "") {
      try {
        await api.folder.create(project, folder_name, location);
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
    } else {
      warning("No file is currently selected.");
    }

    // if (currentFile === file.path) {
    //   setForcedSaveCurrent(!forceSaveCurrent);
    // }
  };

  const handleCloseRenameModal = () => {
    setRenameModalOpen(false);
  };

  const handleSubmitRenameModal = async (new_path: string) => {
    if (renameEntry) {
      try {
        console.log(renameEntry);
        if (renameEntry.is_dir) {
          await api.folder.rename(project, renameEntry.path, new_path);
        } else {
          await api.file.rename(project, renameEntry.path, new_path);
        }

        fetchFileList(); // Update the file list

        if (currentFile && currentFile.path === renameEntry.path) {
          currentFile.path = new_path;
          setCurrentFile(currentFile); // Unset the current file
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
    if (currentFile) {
      handleRename(currentFile);
    } else {
      warning("No file is currently selected.");
    }
  };

  ///////////////// UPLOAD /////////////////////////////////////////////////////

  const handleUpload = (file: Entry) => {
    updateSelectedLocation(file);
    setUploadModalOpen(true);
  };

  const handleCloseUploadModal = () => {
    setUploadModalOpen(false);
    fetchFileList();
  };

  ///////////////// DOWNLOAD ///////////////////////////////////////////////////
  const zipFile = async (zip: JSZip, file_path: string, file_name: string) => {
    var content = await api.file.get(project, file_path);
    zip.file(file_name, content);
  };

  const zipFolder = async (zip: JSZip, file: Entry) => {
    const folder = zip.folder(file.name);

    if (folder === null) {
      return;
    }

    for (let index = 0; index < file.files.length; index++) {
      const element = file.files[index];
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
    <div className="bt-sidebar-content" id={api.name}>
      <div className="bt-sidebar-entry">
        <div className="bt-sidebar-entry-menu">
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
          <div style={{ marginLeft: "auto" }} />
          {currentFile && (
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
          setCurrentFile={setCurrentFile}
          currentFile={currentFile}
          currentProjectname={project}
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
      {renameEntry && (
        <RenameModal
          isOpen={isRenameModalOpen}
          onSubmit={handleSubmitRenameModal}
          onClose={handleCloseRenameModal}
          fileList={fileList}
          selectedEntry={renameEntry}
        />
      )}
      <UploadModal
        isOpen={isUploadModalOpen}
        onSubmit={handleCloseUploadModal}
        onClose={handleCloseUploadModal}
        location={selectedLocation}
        currentProject={project}
      />
      {deleteEntry && (
        <DeleteModal
          isOpen={isDeleteModalOpen}
          onSubmit={handleSubmitDeleteModal}
          onClose={handleCloseDeleteModal}
          selectedEntry={deleteEntry}
        />
      )}
    </div>
  );
};

export default Explorer;
