import React, { useEffect, useState } from "react";
import axios from "axios";
import "./FileBrowser.css";
import NewFileModal from "./modals/NewFileModal.jsx";
import RenameModal from "./modals/RenameModal.jsx";
import NewFolderModal from "./modals/NewFolderModal.jsx";
import UploadModal from "./modals/UploadModal.tsx";
import DeleteModal from "./modals/DeleteModal.jsx";
import FileExplorer from "./file_explorer/FileExplorer.jsx";

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
        const response = await axios.get(
          `/bt_studio/get_file_list?project_name=${currentProjectname}`,
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
  const fetchDownloadData = async (file_path) => {
    const api_response = await fetch("/bt_studio/download_data/", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        app_name: currentProjectname,
        path: file_path,
      }),
    });

    if (!api_response.ok) {
      var json_response = await api_response.json();
      throw new Error(json_response.message || "An error occurred.");
    }

    return api_response.blob();
  };

  const handleDownload = async (file) => {
    if (file) {
      // Get the data as a base64 blob object
      const app_blob = await fetchDownloadData(file.path);

      try {
        const url = window.URL.createObjectURL(app_blob);
        const a = document.createElement("a");
        a.style.display = "none";
        a.href = url;
        a.download = `${file.name}.zip`;
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url);
      } catch (error) {
        console.error("Error:", error);
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
