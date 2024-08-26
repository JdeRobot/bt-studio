import React, { useState, useEffect, useRef } from "react";
import axios from "axios";
import JSZip from "jszip";

import "./UploadModal.css";
import Modal from "../../Modal/Modal";
import ProgressBar from "../../progress_bar/ProgressBar";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const UploadModal = ({
  onSubmit,
  isOpen,
  onClose,
  location,
  currentProject,
}) => {
  const [uploadStatus, setUploadStatus] = useState("");
  const [uploadPercentage, setUploadPercentage] = useState(0);

  const uploadInputRef = useRef(null);
  const uploadAreaRef = useRef(null);

  useEffect(() => {
    setUploadStatus("");
    setUploadPercentage(0);
    uploadInputRef.current.value = "";
  }, [isOpen]);

  const handleDrop = (event) => {
    event.preventDefault();
    uploadAreaRef.current.classList.remove("drag-active");

    if (event.dataTransfer.files.length > 0) {
      uploadInputRef.current.files = event.dataTransfer.files;
      handleAcceptedFiles(uploadInputRef.current.files);
    }
  };

  const onZipUpdate = (metadata) => {
    setUploadPercentage(metadata.percent);
  };

  const handleAcceptedFiles = async (files) => {
    // TODO: Redo for directory
    handleZipFiles(Array.from(files));
  };

  const handleZipFiles = async (file_array) => {
    // TODO: check if files are valid
    const zip = new JSZip();

    file_array.forEach((file, index) => {
      zip.file(file.name, file);
    });

    const zipContent = await zip.generateAsync({ type: "base64" }, onZipUpdate);

    try {
      uploadFileToBackend(zipContent);
      console.log("Uploading file Completed");
    } catch (error) {
      console.log(error);
      console.log("Error uploading file");
    }
  };

  const uploadFileToBackend = async (uploadedData) => {
    console.log("Calling the saving API");
    console.log(currentProject);

    try {
      const response = await axios.post("/tree_api/upload_code/", {
        project_name: currentProject,
        zip_file: uploadedData,
        location: location,
      });
      if (response.data.success) {
        console.log("Universe saved successfully.");
      }
    } catch (error) {
      console.error("Axios Error:", error);
    }

    onClose();
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit();
  };

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
  };

  return (
    <Modal
      id="upload-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label
            className="modal-titlebar-title"
            htmlFor="uploadName"
            style={{ textAlign: "center" }}
          >
            Upload
          </label>
          <CloseIcon
            className="modal-titlebar-close icon"
            onClick={() => {
              handleCancel();
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="form-row">
          <label
            ref={uploadAreaRef}
            htmlFor="uploadDropInput"
            className="modal-drop-container"
            onDragOver={(e) => {
              e.preventDefault();
            }}
            onDragEnter={() =>
              uploadAreaRef.current.classList.add("drag-active")
            }
            onDragLeave={() =>
              uploadAreaRef.current.classList.remove("drag-active")
            }
            onDrop={(e) => handleDrop(e)}
          >
            <span className="modal-drop-title">Drop files here</span>
            or
            <input
              ref={uploadInputRef}
              className="modal-button"
              id="uploadDropInput"
              onChange={(e) => handleAcceptedFiles(e.target.files)}
              type="file"
              title="Upload folder contents"
              webkitdirectory="true"
              multiple
              required
            />
          </label>
        </div>
        {uploadStatus !== "" && (
          <ProgressBar completed={uploadPercentage} />
          // <div className="form-row">
          // </div>
        )}
      </form>
    </Modal>
  );
};

export default UploadModal;
