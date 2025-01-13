import React, { useState, useEffect, useRef } from "react";

import "./UploadModal.css";
import Modal from "../../Modal/Modal";
import ProgressBar from "../../progress_bar/ProgressBar";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";
import { uploadFile } from "../../../api_helper/TreeWrapper";
import { useError } from "../../error_popup/ErrorModal";
import { Entry } from "../FileBrowser";

const UploadModal = ({
  onSubmit,
  isOpen,
  onClose,
  location,
  currentProject,
}: {
  onSubmit: () => void;
  isOpen: boolean;
  onClose: Function;
  location: string;
  currentProject: string;
}) => {
  const { error } = useError();

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

  const handleAcceptedFiles = async (files: FileList | null) => {
    // TODO: Redo for directory
    if (files) {
      handleZipFiles(Array.from(files));
    }
  };

  const handleZipFiles = async (file_array: File[]) => {
    // TODO: check if files are valid
    const n_files = file_array.length;
    var n_files_uploaded = 0;

    file_array.forEach((file: File, index: number) => {
      var reader = new FileReader();

      reader.onprogress = (data) => {
        if (data.lengthComputable) {
          const progress = Math.round((data.loaded / data.total) * 100);
          setUploadPercentage(progress * (n_files_uploaded / n_files));
        }
      };

      reader.onload = (e: any) => {
        const base64String = e.target.result.split(",")[1]; // Remove the data URL prefix
        try {
          uploadFile(currentProject, file.name, location, base64String);
          console.log("Uploading file Completed");
        } catch (e) {
          if (e instanceof Error) {
            console.error("Error uploading file" + e.message);
            error("Error uploading file" + e.message);
          }
        }

        setUploadStatus("Uploaded");
        setUploadPercentage(100 * (n_files_uploaded / n_files));
      };

      reader.readAsDataURL(file);
      n_files_uploaded++;
    });

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
