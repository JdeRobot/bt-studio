import React, { useEffect, useState } from 'react';
import AceEditor from 'react-ace';
import axios from 'axios';
import 'ace-builds/src-noconflict/mode-python';
import 'ace-builds/src-noconflict/theme-monokai';
import './FileEditor.css'

const FileEditor = ({ currentFilename }) => {

  const [fileContent, setFileContent] = useState(""); // To store file content
  const [fontSize, setFontSize] = useState(14); // To store current font size

  useEffect(() => {
    if (currentFilename !== "") {
      axios.get(`/tree_api/get_file?filename=${currentFilename}`)
        .then(response => {
          const content = response.data.content;
          setFileContent(content);
        })
        .catch(error => {
          console.error('Error fetching file content:', error);
        });
    }
  }, [currentFilename]);

  const handleZoomIn = () => {
    setFontSize(prevFontSize => prevFontSize + 2);
  };

  const handleZoomOut = () => {
    setFontSize(prevFontSize => Math.max(10, prevFontSize - 2));
  };

  return (
    <div className="editor-container">
      <h2>File Editor</h2>
      <div className="zoom-buttons">
        <button onClick={handleZoomIn}>+</button>
        <button onClick={handleZoomOut}>-</button>
      </div>
      <AceEditor
        mode="python"
        theme="monokai"
        name="fileEditor"
        width="100%"
        height="80vh"
        value={fileContent}
        fontSize={fontSize}
      />
    </div>
  );
};

export default FileEditor;
