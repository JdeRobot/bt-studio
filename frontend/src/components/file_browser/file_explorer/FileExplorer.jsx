import React, { useEffect, useState } from "react";
import axios from "axios";

import "./FileExplorer.css";
import TreeNode from "./TreeNode.jsx";

const FileExplorer = ({
  setCurrentFilename,
  currentFilename,
  currentProjectname,
  actionNodesData,
  showAccentColor,
  diagramEditorReady,
}) => {
  const [fileList, setFileList] = useState(null);

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

  const handleFileClick = (filename) => {
    setCurrentFilename(filename);
  };

  const handleDeleteFile = async () => {
    if (currentFilename) {
      try {
        const response = await axios.get(
          `/tree_api/delete_file?project_name=${currentProjectname}&filename=${currentFilename}`
        );
        if (response.data.success) {
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

  if (Array.isArray(fileList)) {
    return (
      <div>
        {fileList.map((file) => (
          <TreeNode
            node={file}
            depth={0}
            currentFilename={currentFilename}
            showAccentColor={showAccentColor}
            diagramEditorReady={diagramEditorReady}
            actionNodesData={actionNodesData}
            handleFileClick={handleFileClick}
          />
        ))}
      </div>
    );
  } else {
    return (
      <>
        <p>Create or select a project to start</p>
      </>
    );
  }
};

export default FileExplorer;
