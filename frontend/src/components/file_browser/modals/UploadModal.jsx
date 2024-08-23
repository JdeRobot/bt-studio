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
  // const [uploadedData, setUploadedData] = useState("");
  const [uploadStatus, setUploadStatus] = useState("");
  const [uploadPercentage, setUploadPercentage] = useState(0);

  const uploadInputRef = useRef(null);
  const uploadAreaRef = useRef(null);

  useEffect(() => {
    setUploadStatus("");
    // setUploadedData("");
    setUploadPercentage(0);
    uploadInputRef.current.value = "";
  }, [isOpen]);

  const handleDrop = (event) => {
    event.preventDefault();
    uploadAreaRef.current.classList.remove("drag-active");
    uploadInputRef.current.files = event.dataTransfer.files;
    handleFileReader(uploadInputRef.current.files);
  };

  const handleFileReader = (fileList) => {
    var zip = new JSZip();
    setUploadStatus("Uploading");
    let reader = new FileReader();
    console.log(fileList);
    // TODO: Redo for directory and multiple files
    let file = fileList[0];
    let fileName = file.name.toString().split(".")[0];

    reader.readAsDataURL(file);

    reader.onloadstart = () => {
      setUploadPercentage(0);
    };

    reader.onprogress = (data) => {
      console.log(data);
      if (data.lengthComputable) {
        const progress = Math.round((data.loaded / data.total) * 100);
        setUploadPercentage(progress);
      }
    };

    reader.onload = (e) => {
      console.log("Loaded!");
      const base64String = e.target.result.split(",")[1]; // Remove the data URL prefix
      setUploadStatus("Uploaded");
      setUploadPercentage(100);
      console.log(atob(base64String));
      zip.file(fileName, atob(base64String));
      zip.generateAsync({ type: "base64" }).then(function (blob) {
        uploadFileToBackend(blob);
      });
    };

    reader.onerror = () => {
      setUploadStatus("Error");
      setUploadPercentage(0);
    };
  };

  const uploadFileToBackend = async (uploadedData) => {
    console.log("Calling the saving API");
    console.log(currentProject);

    // if (uploadPercentage != 100) {
    //   console.warn("Not yet uploaded!");
    //   return;
    // }

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
              onChange={(e) => handleFileReader(e.target.files)}
              type="file"
              title="Upload file or folder"
              webkitdirectory
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
