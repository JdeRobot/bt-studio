import React, { useEffect, useState } from "react";
import axios from "axios";

import "./FileExplorer.css";

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
        {fileList.map((file, index) => (
          <div
            key={index}
            className={`file-item ${currentFilename === file + ".py" ? "file-item-selected" : ""}`}
            onClick={() => handleFileClick(file)}
          >
            <label>{file}</label>
            {showAccentColor && diagramEditorReady && (
              <div
                className="accent-color"
                style={{
                  backgroundColor: actionNodesData[file]
                    ? actionNodesData[file]["color"]
                    : "none",
                }}
              />
            )}
          </div>
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
