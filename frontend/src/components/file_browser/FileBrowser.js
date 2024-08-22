import React, { useEffect, useState } from "react";
import axios from "axios";
import "./FileBrowser.css";
import NewActionModal from "./modals/NewActionModal.jsx";
import NewFolderModal from "./modals/NewFolderModal.jsx";
import UploadModal from "./modals/UploadModal.jsx";
import DeleteModal from "./modals/DeleteModal.jsx";
import FileExplorer from "./file_explorer/FileExplorer.jsx";

import { ReactComponent as AddIcon } from "./img/add.svg";
import { ReactComponent as AddFolderIcon } from "./img/add_folder.svg";
import { ReactComponent as DeleteIcon } from "./img/delete.svg";

function getParentDir(file) {
  // Check if is a directory and if not get the parent directory of the file
  if (file.is_dir) {
    return file.path;
  }

  var split_path = file.path.split("/"); // TODO: add for windows
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
}) => {
  const [fileList, setFileList] = useState(null);
  const [isNewActionModalOpen, setNewActionModalOpen] = useState(false);
  const [isNewFolderModalOpen, setNewFolderModalOpen] = useState(false);
  const [isDeleteModalOpen, setDeleteModalOpen] = useState(false);
  const [isUploadModalOpen, setUploadModalOpen] = useState(false);
  const [newsletterFormData, setNewsletterFormData] = useState(null);
  const [selectedEntry, setSelectedEntry] = useState(null);
  const [deleteEntry, setDeleteEntry] = useState(null);
  const [selectedLocation, setSelectedLocation] = useState("");

  useEffect(() => {
    if (selectedEntry) {
      setSelectedLocation(getParentDir(selectedEntry));
    } else {
      setSelectedLocation("");
    }
  }, [selectedEntry]);

  const fetchFileList = async () => {
    if (currentProjectname !== "") {
      try {
        const response = await axios.get(
          `/tree_api/get_file_list?project_name=${currentProjectname}`
        );
        const files = JSON.parse(response.data.file_list);
        setFileList(files);
        // if (Array.isArray(files)) {
        // } else {
        //   console.error("API response is not an array:", files);
        // }
      } catch (error) {
        console.error("Error fetching files:", error);
      }
    }
  };

  ///////////////// CREATE FILES ///////////////////////////////////////////////

  const handleCreateFile = () => {
    // TODO: something similar to the folder one to handle location selected
    setNewActionModalOpen(true);
  };

  const handleCloseNewActionModal = () => {
    setNewActionModalOpen(false);
    document.getElementById("actionName").value = "";
  };

  const handleNewActionSubmit = async (data) => {
    setNewsletterFormData(data);
    handleCloseNewActionModal();

    if (data.actionName !== "") {
      try {
        const response = await axios.get(
          `/tree_api/create_file?project_name=${currentProjectname}&filename=${data.actionName}.py&template=${data.templateType}`
        );
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

  const handleDeleteModal = (file_path) => {
    if (file_path) {
      setDeleteEntry(file_path);
      setDeleteModalOpen(true);
    } else {
      alert("No file is currently selected.");
    }
  };

  const handleCloseDeleteModal = () => {
    setDeleteModalOpen(false);
    setDeleteEntry("");
  };

  const handleSubmitDeleteModal = async () => {
    //currentFilename === Absolute File path
    if (deleteEntry) {
      try {
        const response = await axios.get(
          `/tree_api/delete_file?project_name=${currentProjectname}&path=${deleteEntry}`
        );
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
          if (currentFilename === deleteEntry) {
            setCurrentFilename(""); // Unset the current file
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
      handleDeleteModal(currentFilename);
    } else {
      alert("No file is currently selected.");
    }
  };

  ///////////////// CREATE FOLDER //////////////////////////////////////////////

  const handleCreateFolder = (file) => {
    if (file) {
      setSelectedLocation(getParentDir(file));
    } else {
      if (selectedEntry) {
        setSelectedLocation(getParentDir(selectedEntry));
      } else {
        setSelectedLocation("");
      }
    }

    // TODO: Open modal to ask for the name
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
          `/tree_api/create_folder?project_name=${currentProjectname}&location=${location}&folder_name=${folder_name}`
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

  ///////////////// UPLOAD /////////////////////////////////////////////////////

  const handleUpload = () => {
    // TODO: something similar to the folder one to handle location selected
    setUploadModalOpen(true);
  };

  const handleCloseUploadModal = () => {
    setUploadModalOpen(false);
  };

  return (
    <div style={{ flex: "2" }}>
      <div className="browser-menu">
        {/* <h2>File Explorer</h2> */}
        {/* Add them in a row below or smaller */}
        <div className="buttons">
          <button
            className="menu-button"
            onClick={handleCreateFile}
            title="Create a new action file"
          >
            <AddIcon className="icon" fill={"var(--icon)"} />
          </button>
          <button
            className="menu-button"
            onClick={() => handleCreateFolder(null)}
            title="Create a new folder"
          >
            <AddFolderIcon className="icon" fill={"var(--icon)"} />
          </button>
          {currentFilename !== "" && (
            <button
              className="menu-button"
              onClick={handleDeleteCurrentFile}
              title="Delete file"
            >
              <DeleteIcon className="icon" fill={"var(--icon)"} />
            </button>
          )}
        </div>
      </div>
      <NewActionModal
        isOpen={isNewActionModalOpen}
        onSubmit={handleNewActionSubmit}
        onClose={handleCloseNewActionModal}
        fileList={fileList}
      />
      <NewFolderModal
        isOpen={isNewFolderModalOpen}
        onSubmit={handleCreateFolderSubmit}
        onClose={handleCloseCreateFolder}
        fileList={fileList}
        location={selectedLocation}
      />
      <UploadModal
        isOpen={isUploadModalOpen}
        onSubmit={handleCloseUploadModal}
        onClose={handleCloseUploadModal}
        selectedEntry={selectedEntry}
      />
      <DeleteModal
        isOpen={isDeleteModalOpen}
        onSubmit={handleSubmitDeleteModal}
        onClose={handleCloseDeleteModal}
        selectedEntry={deleteEntry}
        currentProjectname={currentProjectname}
      />
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
      />
    </div>
  );
};

export default FileBrowser;
