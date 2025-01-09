import React, { useEffect, useState } from "react";
import JSZip from "jszip";
import axios from "axios";
import "./FileBrowser.css";
import NewFileModal from "./modals/NewFileModal.jsx";
import RenameModal from "./modals/RenameModal.jsx";
import NewFolderModal from "./modals/NewFolderModal.jsx";
import UploadModal from "./modals/UploadModal.tsx";
import DeleteModal from "./modals/DeleteModal.jsx";
import FileExplorer from "./file_explorer/FileExplorer.jsx";

import { getFile, getFileList } from "./../../api_helper/TreeWrapper";

import { ReactComponent as AddIcon } from "./img/add.svg";
import { ReactComponent as AddFolderIcon } from "./img/add_folder.svg";
import { ReactComponent as DeleteIcon } from "./img/delete.svg";
import { ReactComponent as RefreshIcon } from "./img/refresh.svg";
import { ReactComponent as RenameIcon } from "./img/rename.svg";

function getParentDir(file) {
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
  actionNodesData,
  showAccentColor,
  diagramEditorReady,
  setAutosave,
  forceSaveCurrent,
  setForcedSaveCurrent,
}) => {
  const [fileList, setFileList] = useState(null);
  const [isNewFileModalOpen, setNewFileModalOpen] = useState(false);
  const [isNewFolderModalOpen, setNewFolderModalOpen] = useState(false);
  const [isRenameModalOpen, setRenameModalOpen] = useState(false);
  const [isDeleteModalOpen, setDeleteModalOpen] = useState(false);
  const [isUploadModalOpen, setUploadModalOpen] = useState(false);
  const [selectedEntry, setSelectedEntry] = useState(null);
  const [deleteEntry, setDeleteEntry] = useState(null);
  const [deleteType, setDeleteType] = useState(false);
  const [renameEntry, setRenameEntry] = useState(null);
  const [selectedLocation, setSelectedLocation] = useState("");

  useEffect(() => {
    updateSelectedLocation(null);
  }, [selectedEntry]);

  const updateSelectedLocation = (file) => {
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
      } catch (error) {
        console.error("Error fetching files:", error);
      }
    }
  };

  ///////////////// CREATE FILES ///////////////////////////////////////////////

  const handleCreateFile = (file) => {
    updateSelectedLocation(file);
    setNewFileModalOpen(true);
  };

  const handleCloseNewFileModal = () => {
    setNewFileModalOpen(false);
    document.getElementById("fileName").value = "";
  };

  const handleNewActionSubmit = async (location, data) => {
    handleCloseNewFileModal();

    if (data.fileName !== "") {
      try {
        let response;
        switch (data.fileType) {
          case "actions":
            response = await axios.get(
              `/bt_studio/create_action?project_name=${currentProjectname}&filename=${data.fileName}.py&template=${data.templateType}`,
            );
            break;
          default:
            response = await axios.get(
              `/bt_studio/create_file?project_name=${currentProjectname}&location=${location}&file_name=${data.fileName}`,
            );
            break;
        }
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
        } else {
          alert(response.data.message);
        }
      } catch (error) {
        console.error("Error creating file:", error);
      }
    }
  };

  ///////////////// DELETE FILES AND FOLDERS ///////////////////////////////////

  const handleDeleteModal = (file_path, is_dir) => {
    if (file_path) {
      setDeleteEntry(file_path);
      setDeleteType(is_dir);
      setDeleteModalOpen(true);
    } else {
      alert("No file is currently selected.");
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
        var response;
        if (deleteType) {
          response = await axios.get(
            `/bt_studio/delete_folder?project_name=${currentProjectname}&path=${deleteEntry}`,
          );
        } else {
          response = await axios.get(
            `/bt_studio/delete_file?project_name=${currentProjectname}&path=${deleteEntry}`,
          );
        }
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
          if (currentFilename === deleteEntry) {
            setCurrentFilename(""); // Unset the current file
          }
          if (selectedEntry.path === deleteEntry) {
            setSelectedEntry(null);
          }
        } else {
          alert(response.data.message);
        }
      } catch (error) {
        console.error("Error deleting file:", error);
      }
    } else {
      alert("No file is currently selected.");
    }
    handleCloseDeleteModal();
  };

  const handleDeleteCurrentFile = () => {
    //currentFilename === Absolute File path
    if (currentFilename) {
      handleDeleteModal(currentFilename, false);
      setAutosave(false);
    } else {
      alert("No file is currently selected.");
    }
  };

  ///////////////// CREATE FOLDER //////////////////////////////////////////////

  const handleCreateFolder = (file) => {
    updateSelectedLocation(file);
    setNewFolderModalOpen(true);
  };

  const handleCloseCreateFolder = () => {
    setNewFolderModalOpen(false);
    document.getElementById("folderName").value = "";
  };

  const handleCreateFolderSubmit = async (location, folder_name) => {
    if (folder_name !== "") {
      try {
        const response = await axios.get(
          `/bt_studio/create_folder?project_name=${currentProjectname}&location=${location}&folder_name=${folder_name}`,
        );
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
        } else {
          alert(response.data.message);
        }
      } catch (error) {
        console.error("Error creating folder:", error);
      }
    }
  };

  ///////////////// RENAME /////////////////////////////////////////////////////

  const handleRename = (file) => {
    if (file) {
      setRenameEntry(file);
      setRenameModalOpen(true);
    } else {
      alert("No file is currently selected.");
    }

    if (currentFilename === file.path) {
      setForcedSaveCurrent(!forceSaveCurrent);
    }
  };

  const handleCloseRenameModal = () => {
    setRenameModalOpen(false);
  };

  const handleSubmitRenameModal = async (new_path) => {
    if (renameEntry) {
      try {
        var response;
        console.log(renameEntry);
        if (renameEntry.is_dir) {
          response = await axios.get(
            `/bt_studio/rename_folder?project_name=${currentProjectname}&path=${renameEntry.path}&rename_to=${new_path}`,
          );
        } else {
          response = await axios.get(
            `/bt_studio/rename_file?project_name=${currentProjectname}&path=${renameEntry.path}&rename_to=${new_path}`,
          );
        }
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
          if (currentFilename === renameEntry.path) {
            setAutosave(false);
            setCurrentFilename(new_path); // Unset the current file
          }
        } else {
          alert(response.data.message);
        }
      } catch (error) {
        console.error("Error deleting file:", error);
      }
    } else {
      alert("No file is currently selected.");
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
      alert("No file is currently selected.");
    }
  };

  ///////////////// UPLOAD /////////////////////////////////////////////////////

  const handleUpload = (file) => {
    updateSelectedLocation(file);
    setUploadModalOpen(true);
  };

  const handleCloseUploadModal = () => {
    setUploadModalOpen(false);
    fetchFileList();
  };

  ///////////////// DOWNLOAD ///////////////////////////////////////////////////
  const zipFile = async (zip, file_path, file_name) => {
    var content = await getFile(currentProjectname, file_path);
    zip.file(file_name, content);
  };

  const zipFolder = async (zip, file) => {
    const folder = zip.folder(file.name);

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

  const handleDownload = async (file) => {
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
      } catch (error) {
        console.error("Error downloading file: " + error);
      }
    }
  };

  return (
    <div className="bt-sidebar-content">
      <div className="bt-sidebar-entry">
        <div className="bt-sidebar-entry-menu">
          <button
            className="bt-sidebar-button"
            onClick={() => handleCreateFile(null)}
            title="Create a new file"
          >
            <AddIcon className="bt-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-sidebar-button"
            onClick={() => handleCreateFolder(null)}
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
          actionNodesData={actionNodesData}
          showAccentColor={showAccentColor}
          diagramEditorReady={diagramEditorReady}
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
