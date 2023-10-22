import React, { useEffect, useState } from 'react';
import AceEditor from 'react-ace';
import axios from 'axios';
import 'ace-builds/src-noconflict/mode-python';
import 'ace-builds/src-noconflict/theme-monokai';
import './FileEditor.css'

import save_img from './img/save.svg' 

const FileEditor = ({ currentFilename }) => {
  
  const [fileContent, setFileContent] = useState("");
  const [fontSize, setFontSize] = useState(14);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);

  useEffect(() => {
    if (currentFilename) {
      axios.get(`/tree_api/get_file?filename=${currentFilename}`)
        .then(response => {
          const content = response.data.content;
          setFileContent(content);
          setHasUnsavedChanges(false); // Reset the unsaved changes flag when a new file is loaded
        })
        .catch(error => {
          console.error('Error fetching file content:', error);
        });
    }
    else {
      setFileContent("");
      setHasUnsavedChanges(false);
    }
  }, [currentFilename]);

  const handleSaveFile = () => {
    if (currentFilename) {
      axios.post('/tree_api/save_file/', {
        filename: currentFilename,
        content: fileContent
      })
      .then(response => {
        if (response.data.success) {
          alert("File saved successfully.");
          setHasUnsavedChanges(false); // Reset the unsaved changes flag
        } else {
          alert(`Failed to save file: ${response.data.message}`);
        }
      })
      .catch(error => {
        console.error('Error saving file:', error);
      });
    } else {
      alert("No file is currently selected.");
    }
  };

  const handleZoomIn = () => {
    setFontSize(prevFontSize => prevFontSize + 2);
  };

  const handleZoomOut = () => {
    setFontSize(prevFontSize => Math.max(10, prevFontSize - 2));
  };

  return (
    <div className="editor-container">
      <div className='editor-menu'>
        <h2>File Editor</h2>
        <div className="editor-buttons">
          {hasUnsavedChanges && <div className="unsaved-dot"></div>}
          <button className="save-button" onClick={handleSaveFile}>
            <img className="icon" src={save_img}></img>
          </button>
        </div>
      </div>
      <div className="zoom-buttons">
        <button className="zoom-in" onClick={handleZoomIn}>+</button>
        <button className="zoom-in" onClick={handleZoomOut}>-</button>
      </div>
      <AceEditor
        mode="python"
        theme="monokai"
        name="fileEditor"
        width="100%"
        height="80vh"
        value={fileContent}
        fontSize={fontSize}
        onChange={newContent => {
          setFileContent(newContent);
          setHasUnsavedChanges(true); // Set the unsaved changes flag
        }}/>
    </div>
  );
};

export default FileEditor;
