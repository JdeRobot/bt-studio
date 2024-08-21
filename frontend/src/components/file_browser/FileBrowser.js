import React, { useEffect, useState } from "react";
import axios from "axios";
import "./FileBrowser.css";
import NewActionModal from "./NewActionModal.jsx";
import FileExplorer from "./file_explorer/FileExplorer.jsx";

import { ReactComponent as AddIcon } from "./img/add.svg";
import { ReactComponent as DeleteIcon } from "./img/delete.svg";

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
  const [newsletterFormData, setNewsletterFormData] = useState(null);
  const [selectedEntry, setSelectedEntry] = useState("");

  const fetchFileList = async () => {
    if (currentProjectname !== "") {
      try {
        const response = await axios.get(
          `/tree_api/get_file_list?project_name=${currentProjectname}`,
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

  const handleCreateFile = () => {
    // TODO: something similar to the folder one to handle location selected
    setNewActionModalOpen(true);
  };

  const handleCreateFolder = (file) => {
    let path;

    if (file) {
      path = file.path;
      // Check if is a directory and if not get the parent directory of the file
      if (!file.is_dir) {
        var split_path = file.path.split("/"); // TODO: add for windows
        path = split_path.slice(0, split_path.length - 1).join("/");
      }
    } else {
      path = selectedEntry;
    }

    // TODO: Open modal to ask for the name
    handleCreateFolderCall(path, "");
  };

  const handleCreateFolderCall = async (location, folder_name) => {
    if (folder_name !== "") {
      try {
        const response = await axios.get(
          `/tree_api/create_folder?project_name=${currentProjectname}&location=${location}&folder_name=${folder_name}`,
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

  const handleDeleteFile = async (file_path) => {
    //currentFilename === Absolute File path
    if (file_path) {
      try {
        const response = await axios.get(
          `/tree_api/delete_file?project_name=${currentProjectname}&path=${file_path}`,
        );
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
          if (currentFilename === file_path) {
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
  };

  const handleDeleteCurrentFile = async () => {
    //currentFilename === Absolute File path
    if (currentFilename) {
      await handleDeleteFile(currentFilename);
    } else {
      alert("No file is currently selected.");
    }
  };

  const handleCloseNewActionModal = () => {
    setNewActionModalOpen(false);
    document.getElementById("actionName").value = "";
  };

  const handleFormSubmit = async (data) => {
    setNewsletterFormData(data);
    handleCloseNewActionModal();

    if (data.actionName !== "") {
      try {
        const response = await axios.get(
          `/tree_api/create_file?project_name=${currentProjectname}&filename=${data.actionName}.py&template=${data.templateType}`,
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

  return (
    <div style={{ flex: "2" }}>
      <div className="browser-menu">
        <h2>File Explorer</h2>
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
            onClick={handleDeleteCurrentFile}
            title="Delete file"
          >
            <DeleteIcon className="icon" fill={"var(--icon)"} />
          </button>
        </div>
      </div>
      <NewActionModal
        isOpen={isNewActionModalOpen}
        onSubmit={handleFormSubmit}
        onClose={handleCloseNewActionModal}
        fileList={fileList}
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
        onDelete={handleDeleteFile}
        onCreateFile={handleCreateFile}
        onCreateFolder={handleCreateFolder}
      />
    </div>
  );
};

export default FileBrowser;
