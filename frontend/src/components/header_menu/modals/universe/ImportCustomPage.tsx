import React, { useState, useEffect, useRef } from "react";
import "./CreatePage.css";
import "./ImportCustomPage.css";
import { ReactComponent as BackIcon } from "../../../Modal/img/back.svg";
import { ReactComponent as CloseIcon } from "../../../Modal/img/close.svg";
import { uploadUniverse } from "../../../../api_helper/TreeWrapper";
import { useError } from "../../../error_popup/ErrorModal";
import ProgressBar from "../../../progress_bar/ProgressBar";

const initialUniverseData = {
  universeName: "",
};

const ImportCustomPage = ({
  setVisible,
  visible,
  onClose,
  currentProject,
}: {
  setVisible: Function;
  visible: boolean;
  onClose: Function;
  currentProject: string;
}) => {
  const { error } = useError();

  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialUniverseData);
  const [uploadedUniverse, setUploadedUniverse] = useState<string>("");
  const [uploadStatus, setUploadStatus] = useState<string>("");
  const [uploadPercentage, setUploadPercentage] = useState<number>(0);

  const uploadInputRef = useRef<any>(null);
  const uploadAreaRef = useRef<any>(null);

  useEffect(() => {
    if (visible && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [visible]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
  };

  const handleCancel = () => {
    if (currentProject !== "") {
      onClose();
    }
  };

  const handleCreate = async () => {
    if (formState.universeName === "") {
      return;
    }

    if (uploadPercentage !== 100) {
      console.warn("Not yet uploaded!");
      return;
    }

    uploadUniverse(currentProject, formState.universeName, uploadedUniverse);

    setVisible(false);
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
          // uploadFile(currentProject, file.name, location, base64String);
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
  };

  const handleDrop = (event: any) => {
    event.preventDefault();
    uploadAreaRef.current.classList.remove("bt-drag-active");

    if (
      event.dataTransfer.files.length === 1 &&
      event.dataTransfer.files[0].type === "application/zip"
    ) {
      uploadInputRef.current.files = event.dataTransfer.files;
      handleAcceptedFiles(uploadInputRef.current.files[0]);
    }
  };

  return (
    <>
      <div className="bt-modal-titlebar">
        <BackIcon
          className="bt-modal-titlebar-back bt-icon"
          onClick={() => {
            setVisible(false);
          }}
          fill={"var(--icon)"}
        />
        <label
          className="bt-modal-titlebar-title"
          htmlFor="actionName"
          style={{ textAlign: "center" }}
        >
          Create a Universe
        </label>
        <CloseIcon
          className="bt-modal-titlebar-close bt-icon"
          onClick={() => {
            handleCancel();
          }}
          fill={"var(--icon)"}
        />
      </div>
      <div className="bt-modal-complex-input-row-container">
        <div className="bt-universe-create-name bt-modal-complex-input-container">
          <input
            ref={focusInputRef}
            type="text"
            id="universeName"
            name="universeName"
            className="bt-modal-complex-input"
            onChange={handleInputChange}
            autoComplete="off"
            maxLength={20}
            placeholder="Universe Name"
            required={true}
          />
          <label
            htmlFor="universeName"
            className="bt-modal-complex-input-label"
          >
            Universe Name
          </label>
          <label
            htmlFor="universeName"
            className="bt-modal-complex-input-indications"
          >
            A unique name that is used for the universe folder and other
            resources. The name should be in lower case without spaces and
            should not start with a number. The maximum length is 20 characters.
          </label>
        </div>
      </div>
      <div className="bt-form-row">
        <div className="bt-modal-complex-input-row-container">
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
            style={{ height: "300px", width: "75%", marginTop: "30px" }}
          >
            <span className="bt-modal-drop-title">Drop zip here</span>
            or
            <input
              ref={uploadInputRef}
              className="bt-modal-button"
              id="uploadDropInput"
              onChange={(e) => handleAcceptedFiles(e.target.files)}
              type="file"
              accept="application/zip"
              name="img"
              title="Upload zip"
              required
              multiple={false}
            />
          </label>
        </div>
      </div>
      <div className="bt-modal-complex-input-row-container">
        {uploadStatus !== "" && (
          <ProgressBar completed={uploadPercentage} />
          // <div className="bt-form-row">
          // </div>
        )}
      </div>
      <div className="bt-modal-complex-input-row-container">
        <div id="create-new-universe" onClick={() => handleCreate()}>
          Create Universe
        </div>
      </div>
    </>
  );
};

export default ImportCustomPage;
