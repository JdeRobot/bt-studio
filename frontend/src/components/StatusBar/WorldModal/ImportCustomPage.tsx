import React, { useState, useEffect, useRef } from "react";
import "./ImportCustomPage.css";
import {
  createEmptyWorld,
  createWorldConfig,
  createWorldFolder,
  uploadFileWorld,
} from "BtApi/TreeWrapper";
import JSZip from "jszip";
import {
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
  ProgressBar,
  ModalInputDropArea,
  useError,
} from "jderobot-ide-interface";

const initialWorldData = {
  worldName: "",
};

const ImportCustomPage = ({
  setVisible,
  visible,
  onClose,
  currentProject,
}: {
  setVisible: (visible: boolean) => void;
  visible: boolean;
  onClose: Function;
  currentProject: string;
}) => {
  const { error } = useError();

  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialWorldData);
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
    if (formState.worldName === "") {
      return;
    }

    await createEmptyWorld(currentProject, formState.worldName);
    await handleAcceptedFiles(uploadInputRef.current.files);
    try {
      await createWorldConfig(currentProject, formState.worldName);
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
      let n_files_uploaded = 0;

      for (const index in files) {
        const filename = files[index];
        const file = zip.files[filename];
        try {
          if (file.dir) {
            await createWorldFolder(
              currentProject,
              filename,
              "",
              formState.worldName,
            );
          } else {
            await file.async("base64").then(async function (fileData) {
              await uploadFileWorld(
                currentProject,
                filename,
                "",
                fileData,
                formState.worldName,
              );
              console.log("Uploading file Completed");
            });
          }
        } catch (e: unknown) {
          if (e instanceof Error) {
            error(e.message);
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
        title="Create a World"
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
          id="worldName"
          placeholder="World Name"
          onChange={handleInputChange}
          description="A unique name that is used for the world folder and other
            resources. The name should be in lower case without spaces and
            should not start with a number. The maximum length is 20 characters."
          type="text"
          autoComplete="off"
          required
          maxLength={20}
        />
      </ModalRow>
      <ModalRow>
        <ModalInputDropArea
          areaRef={uploadAreaRef}
          inputRef={uploadInputRef}
          id="uploadDropInput"
          dropTitle={"Drop zip here"}
          onChange={() => {}}
          onDrop={handleDrop}
          type="file"
          accept="application/zip"
          required
          multiple={false}
        />
      </ModalRow>
      <ProgressBar completed={uploadPercentage} />
      <ModalRow type="buttons">
        <button
          type="button"
          id="create-new-world"
          onClick={() => handleCreate()}
        >
          Create World
        </button>
      </ModalRow>
    </>
  );
};

export default ImportCustomPage;
