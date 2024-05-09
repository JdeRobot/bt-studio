import React, { useEffect, useState } from 'react';
import axios from 'axios';
import './FileBrowser.css';

import add_img from './img/add.svg'
import delete_img from './img/delete.svg'

const FileBrowser = ({ setCurrentFilename, currentFilename, currentProjectname, setProjectChanges}) => {

  const [fileList, setFileList] = useState(null);

  useEffect(() => {
    fetchFileList();
  }, [currentProjectname]);

  const fetchFileList = () => {

    if (currentProjectname != '') {
      axios.get(`/tree_api/get_file_list?project_name=${currentProjectname}`)
        .then(response => {
          const files = response.data.file_list;
          if (Array.isArray(files)) {
            setFileList(files);
          } else {
            console.error('API response is not an array:', files);
          }
        })
        .catch(error => {
          console.error('Error fetching files:', error);
        });
    }
  };

  const handleFileClick = (filename) => {
    setCurrentFilename(filename);
  };

  const handleCreateFile = () => {

    const filename = prompt("Enter new action name:");
    if (filename) {
      axios.get(`/tree_api/create_file?project_name=${currentProjectname}&filename=${filename}`)
        .then(response => {
          if (response.data.success) {
            setProjectChanges(true);
            fetchFileList();  // Update the file list
          } else {
            alert(response.data.message);
          }
        })
        .catch(error => {
          console.error('Error creating file:', error);
        });
    }
  };

  const handleDeleteFile = () => {

    if (currentFilename) {
      axios.get(`/tree_api/delete_file?project_name=${currentProjectname}&filename=${currentFilename}`)
        .then(response => {
          if (response.data.success) {
            setProjectChanges(true);
            fetchFileList();  // Update the file list
            setCurrentFilename(""); // Unset the current file
          } else {
            alert(response.data.message);
          }
        })
        .catch(error => {
          console.error('Error deleting file:', error);
        });
    } else {
      alert("No file is currently selected.");
    }
  };

  return (
    <div style={{ flex: '2', border: '1px solid black' }}>
      <div className='browser-menu'>
        <h2>File Browser</h2>
        <div className='buttons'>
          <button className="menu-button" onClick={handleCreateFile} title='Create a new action file'>
            <img className="icon" src={add_img}></img>
          </button>
          <button className="menu-button" onClick={handleDeleteFile} title='Delete file'>
            <img className="icon" src={delete_img}></img>
          </button>
        </div>
      </div>
      {Array.isArray(fileList) ? (
        <div>
          {fileList.map((file, index) => (
            <div 
              key={index}
              className={`file-item ${currentFilename === file ? 'file-item-selected' : ''}`}
              onClick={() => handleFileClick(file)}
            >
              {file}
            </div>
          ))}
        </div>
      ) : (
        <p>Create or select a project to start</p>
      )}
    </div>
  );
};

export default FileBrowser;
