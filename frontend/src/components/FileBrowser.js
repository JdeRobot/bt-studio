import React, { useEffect, useState } from 'react';
import axios from 'axios';
import './FileBrowser.css';

const FileBrowser = ({ setCurrentFileContent }) => {

  const [fileList, setFileList] = useState(null);

  useEffect(() => {
    axios.get('/tree_api/get_file_list')
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
  }, []);

  const handleFileClick = (filename) => {
    axios.get(`/tree_api/get_file?filename=${filename}`)
      .then(response => {
        const content = response.data.content;
        setCurrentFileContent(content);
      })
      .catch(error => {
        console.error('Error fetching file content:', error);
      });
  };

  return (
    <div style={{ flex: '2', border: '1px solid black' }}>
      <h2>File Editor</h2>
      {Array.isArray(fileList) ? (
        <div>
          {fileList.map((file, index) => (
            <div 
              key={index}
              className="file-item"
              onClick={() => handleFileClick(file)}
            >
              {file}
            </div>
          ))}
        </div>
      ) : (
        <p>Loading or encountered an error...</p>
      )}
    </div>
  );
};

export default FileBrowser;
