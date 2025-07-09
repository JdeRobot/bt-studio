import React, { useState, useEffect, useRef } from "react";
import "./ImportCustomPage.css";
import {
  createEmptyUniverse,
  createUniverseConfig,
  createUniverseFolder,
  uploadFileUniverse,
} from "../../api_helper/TreeWrapper";
import { useError } from "../error_popup/ErrorModal";
import JSZip from "jszip";
import {
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
  ProgressBar,
} from "jderobot-ide-interface";

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
  const [uploadPercentage, setUploadPercentage] = useState(0);

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

    await createEmptyUniverse(currentProject, formState.universeName);
    await handleAcceptedFiles(uploadInputRef.current.files);
    try {
      await createUniverseConfig(currentProject, formState.universeName);
    } catch {
      console.log("Already had configuration");
    }

    setVisible(false);
  };

  const handleAcceptedFiles = async (files: FileList) => {
    if (files) {
      await handleZipFiles(Array.from(files)[0]);
    }
  };

  const handleZipFiles = async (zip: File) => {
    await JSZip.loadAsync(zip).then(async function (zip) {
      const files = Object.keys(zip.files);
      const n_files = files.length;
      var n_files_uploaded = 0;

      for (const index in files) {
        const filename = files[index];
        const file = zip.files[filename];
        try {
          if (file.dir) {
            await createUniverseFolder(
              currentProject,
              filename,
              "",
              formState.universeName,
            );
          } else {
            await file.async("base64").then(async function (fileData) {
              await uploadFileUniverse(
                currentProject,
                filename,
                "",
                fileData,
                formState.universeName,
              );
              console.log("Uploading file Completed");
            });
          }
        } catch (e) {
          if (e instanceof Error) {
            console.error("Error creating folder" + e.message);
            error("Error creating folder" + e.message);
          }
        }
        n_files_uploaded++;
        setUploadPercentage(100 * (n_files_uploaded / n_files));
      }
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
    }
  };

  return (
    <>
      <ModalTitlebar
        title="Create a Universe"
        htmlFor="actionName"
        hasClose
        hasBack
        handleClose={() => {
          handleCancel();
        }}
        handleBack={() => {
          setVisible(false);
        }}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={true}
          ref={focusInputRef}
          id="universeName"
          placeholder="Universe Name"
          onChange={handleInputChange}
          description="A unique name that is used for the universe folder and other
            resources. The name should be in lower case without spaces and
            should not start with a number. The maximum length is 20 characters."
          type="text"
          autoComplete="off"
          required
          maxLength={20}
        />
      </ModalRow>
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
      <ProgressBar completed={uploadPercentage} />
      <ModalRow type="buttons">
        <button
          type="button"
          id="create-new-universe"
          onClick={() => handleCreate()}
        >
          Create Universe
        </button>
      </ModalRow>
    </>
  );
};

export default ImportCustomPage;
