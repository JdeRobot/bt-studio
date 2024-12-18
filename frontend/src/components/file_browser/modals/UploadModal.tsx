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
}: {
  onSubmit: any;
  isOpen: any;
  onClose: any;
  location: any;
  currentProject: any;
}) => {
  const [uploadStatus, setUploadStatus] = useState("");
  const [uploadPercentage, setUploadPercentage] = useState(0);

  const uploadInputRef = useRef<any>(null);
  const uploadAreaRef = useRef<any>(null);

  useEffect(() => {
    setUploadStatus("");
    setUploadPercentage(0);
    uploadInputRef.current.value = "";
  }, [isOpen]);

  const handleDrop = (event: any) => {
    event.preventDefault();
    uploadAreaRef.current.classList.remove("bt-drag-active");

    if (event.dataTransfer.files.length > 0) {
      uploadInputRef.current.files = event.dataTransfer.files;
      handleAcceptedFiles(uploadInputRef.current.files);
    }
  };

  const onZipUpdate = (metadata: any) => {
    setUploadPercentage(metadata.percent);
  };

  const handleAcceptedFiles = async (files: any) => {
    // TODO: Redo for directory
    handleZipFiles(Array.from(files));
  };

  const handleZipFiles = async (file_array: any) => {
    // TODO: check if files are valid
    const zip = new JSZip();

    file_array.forEach((file: any, index: any) => {
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

  const uploadFileToBackend = async (uploadedData: any) => {
    console.log("Calling the saving API");
    console.log(currentProject);

    try {
      const response = await axios.post(
        "/bt_studio/upload_code/",
        {
          project_name: currentProject,
          zip_file: uploadedData,
          location: location,
        },
        {
          headers: {
            //@ts-ignore Needed for compatibility with Unibotics
            "X-CSRFToken": context.csrf,
          },
        },
      );
      if (response.data.success) {
        console.log("Universe saved successfully.");
      }
    } catch (error) {
      console.error("Axios Error:", error);
    }

    onClose();
  };

  const handleSubmit = (event: any) => {
    event.preventDefault();
    onSubmit();
  };

  const handleCancel = (event: any) => {
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
        <div className="bt-modal-titlebar">
          <label
            className="bt-modal-titlebar-title"
            htmlFor="uploadName"
            style={{ textAlign: "center" }}
          >
            Upload
          </label>
          <CloseIcon
            className="bt-modal-titlebar-close bt-icon"
            onClick={(e: any) => {
              handleCancel(e);
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="bt-form-row">
          <label
            ref={uploadAreaRef}
            htmlFor="uploadDropInput"
            className="bt-modal-drop-container"
            onDragOver={(e) => {
              e.preventDefault();
            }}
            onDragEnter={() =>
              uploadAreaRef.current.classList.add("bt-drag-active")
            }
            onDragLeave={() =>
              uploadAreaRef.current.classList.remove("bt-drag-active")
            }
            onDrop={(e) => handleDrop(e)}
          >
            <span className="bt-modal-drop-title">Drop files here</span>
            or
            <input
              ref={uploadInputRef}
              className="bt-modal-button"
              id="uploadDropInput"
              onChange={(e) => handleAcceptedFiles(e.target.files)}
              type="file"
              title="Upload folder contents"
              multiple
              required
            />
          </label>
        </div>
        {uploadStatus !== "" && (
          <ProgressBar completed={uploadPercentage} />
          // <div className="bt-form-row">
          // </div>
        )}
      </form>
    </Modal>
  );
};

export default UploadModal;
