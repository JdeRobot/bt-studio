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

  useEffect(() => {
    fetchFileList();
    setCurrentFilename("");
  }, [currentProjectname]);

  const fetchFileList = async () => {
    if (currentProjectname !== "") {
      try {
        const response = await axios.get(
          `/tree_api/get_file_list?project_name=${currentProjectname}`
        );
        const files = response.data.file_list;
        if (Array.isArray(files)) {
          for (let index = 0; index < files.length; index++) {
            files[index] = files[index].slice(0, -3);
          }
          setFileList(files);
        } else {
          console.error("API response is not an array:", files);
        }
      } catch (error) {
        console.error("Error fetching files:", error);
      }
    }
  };

  const handleFileClick = (filename) => {
    setCurrentFilename(filename + ".py");
  };

  const handleCreateFile = () => {
    setNewActionModalOpen(true);
  };

  const handleDeleteFile = async () => {
    if (currentFilename) {
      try {
        const response = await axios.get(
          `/tree_api/delete_file?project_name=${currentProjectname}&filename=${currentFilename}`
        );
        if (response.data.success) {
          setProjectChanges(true);
          fetchFileList(); // Update the file list
          setCurrentFilename(""); // Unset the current file
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
            onClick={handleDeleteFile}
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
        actionNodesData={actionNodesData}
        showAccentColor={showAccentColor}
        diagramEditorReady={diagramEditorReady}
      />
    </div>
  );
};

export default FileBrowser;
