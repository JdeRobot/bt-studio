import React, { useEffect, useState } from 'react';
import AceEditor from 'react-ace';
import axios from 'axios';
import 'ace-builds/src-noconflict/mode-python';
import 'ace-builds/src-noconflict/theme-monokai';
import './FileEditor.css'

import save_img from './img/save.svg' 
import splash_img from './img/logo_jderobot_monocolor.svg' 

const FileEditor = ({ currentFilename, currentProjectname, setProjectChanges }) => {
  
  const [fileContent, setFileContent] = useState("");
  const [fontSize, setFontSize] = useState(14);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
  const [filenameToSave, setFilenameToSave] = useState('');
  const [projectToSave, setProjectToSave] = useState(currentProjectname);

  useEffect(() => {
    if (currentFilename != '') {
      axios.get(`/tree_api/get_file?project_name=${currentProjectname}&filename=${currentFilename}`)
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
    console.log(filenameToSave);
    // Autosave
    if (filenameToSave) {
      axios.post('/tree_api/save_file/', {
        project_name: currentProjectname,
        filename: filenameToSave,
        content: fileContent
      })
      .then(response => {
        if (response.data.success) {
          setHasUnsavedChanges(false); // Reset the unsaved changes flag
          setProjectChanges(false);
        } else {
          alert(`Failed to save file: ${response.data.message}`);
        }
      })
      .catch(error => {
        console.error('Error saving file:', error);
      });
    }
    setFilenameToSave(currentFilename)
  }, [currentFilename]);

  useEffect(() => {
    setFilenameToSave('');
    if (currentFilename) {
      handleSaveFile();
    }
    setProjectToSave(currentProjectname);
    setFileContent("");
    console.log(currentFilename);
  }, [currentProjectname]);

  const handleSaveFile = () => {
    if (currentFilename) {
      axios.post('/tree_api/save_file/', {
        project_name: projectToSave,
        filename: currentFilename,
        content: fileContent
      })
      .then(response => {
        if (response.data.success) {
          setHasUnsavedChanges(false); // Reset the unsaved changes flag
          setProjectChanges(false);
        } else {
          alert(`Failed to save file: ${response.data.message}`);
        }
      })
      .catch(error => {
        console.error('Error saving file:', error);
      });
    } else {
      alert("No file is currentlyyy selected.");
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
      {fileContent !== "" &&(
        <div className="zoom-buttons">
          <button className="zoom-in" onClick={handleZoomIn}>+</button>
          <button className="zoom-in" onClick={handleZoomOut}>-</button>
        </div>
      )}
      {fileContent !== "" ?(
      <AceEditor
        mode="python"
        theme="monokai"
        name="fileEditor"
        width="100%"
        height="80vh"
        value={fileContent}
        fontSize={fontSize}
        onChange={newContent => {
          setProjectChanges(true);
          setFileContent(newContent);
          setHasUnsavedChanges(true); // Set the unsaved changes flag
        }}/>
      ) : (
        <img className="splash-icon" src={splash_img}></img>
      )
      }
    </div>
  );
};

export default FileEditor;
