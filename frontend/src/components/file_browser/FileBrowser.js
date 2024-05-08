import React, { useEffect, useState } from 'react';
import axios from 'axios';
import './FileBrowser.css';
import NewActionModal from './NewActionModal.jsx';

import add_img from './img/add.svg'
import delete_img from './img/delete.svg'

const FileBrowser = ({ setCurrentFilename, currentFilename, currentProjectname, setProjectChanges}) => {

  const [fileList, setFileList] = useState(null);
  const [isNewActionModalOpen, setNewActionModalOpen] = useState(false);
  const [newsletterFormData, setNewsletterFormData] = useState(null);

  useEffect(() => {
    fetchFileList();
  }, [currentProjectname]);

  const fetchFileList = () => {

    if (currentProjectname !== '') {
      axios.get(`/tree_api/get_file_list?project_name=${currentProjectname}`)
        .then(response => {
          const files = response.data.file_list;
          if (Array.isArray(files)) {
            for (let index = 0; index < files.length; index++) {
              files[index] = files[index].slice(0,-3);
            }
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

    // TODO: new interface
    setNewActionModalOpen(true);
    // const filename = prompt("Enter new action name:");
    // if (filename) {
    //   axios.get(`/tree_api/create_file?project_name=${currentProjectname}&filename=${filename}.py`)
    //     .then(response => {
    //       if (response.data.success) {
    //         setProjectChanges(true);
    //         fetchFileList();  // Update the file list
    //       } else {
    //         alert(response.data.message);
    //       }
    //     })
    //     .catch(error => {
    //       console.error('Error creating file:', error);
    //     });
    // }
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

  const handleCloseNewActionModal = () => {
    setNewActionModalOpen(false);
    document.getElementById('actionName').value = '';
  };

  const handleFormSubmit = (data) => {
    setNewsletterFormData(data);
    console.log(newsletterFormData);
    handleCloseNewActionModal();
    if (data.actionName !== '') {
      axios.get(`/tree_api/create_file?project_name=${currentProjectname}&filename=${data.actionName}.py&template=${data.templateType}`)
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
      <NewActionModal
        isOpen={isNewActionModalOpen}
        onSubmit={handleFormSubmit}
        onClose={handleCloseNewActionModal}
        fileList={fileList}
      />
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
