import { useState } from "react";
import "./UniverseUploadModal.css";
import Modal from "../../Modal/Modal";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";
import { uploadUniverse } from "../../../api_helper/TreeWrapper";
import { useError } from "../../error_popup/ErrorModal";

const initialProjectData = {
  projectName: "",
};

const UniverseUploadModal = ({
  onSubmit,
  isOpen,
  onClose,
  currentProject,
  setUniverseAdded,
}: {
  onSubmit: (data: any) => void;
  isOpen: boolean;
  onClose: Function;
  currentProject: string;
  setUniverseAdded: Function;
}) => {
  const { error } = useError();

  const [formState, setFormState] = useState(initialProjectData);
  const [uploadedUniverse, setUploadedUniverse] = useState<string>("");
  const [uploadStatus, setUploadStatus] = useState<string>("");
  const [uploadPercentage, setUploadPercentage] = useState<number>(0);
  const [universeName, setUniverseName] = useState<string>("");

  const handleCancel = () => {
    if (currentProject !== "") {
      onClose();
    }
  };

  const handleFileReader = (event: any) => {
    setUploadStatus("Uploading");
    let reader = new FileReader();
    let file = event.target.files[0];
    let fileName = file.name.toString().split(".")[0];

    reader.readAsDataURL(file);

    reader.onloadstart = () => {
      setUploadPercentage(0);
    };

    reader.onprogress = (data) => {
      if (data.lengthComputable) {
        const progress = Math.round((data.loaded / data.total) * 100);
        console.log(progress);
        setUploadPercentage(progress);
      }
    };

    reader.onload = (e: any) => {
      console.log("Loaded!");
      const base64String = e.target.result.split(",")[1]; // Remove the data URL prefix
      setUploadedUniverse(base64String);
      setUniverseName(fileName);
      setUploadStatus("Uploaded");
      setUploadPercentage(100);
    };

    reader.onerror = () => {
      setUploadStatus("Error");
      setUploadPercentage(0);
    };
  };

  const saveZipUniverse = async () => {
    console.log("Calling the saving API");

    if (uploadPercentage !== 100) {
      console.warn("Not yet uploaded!");
      return;
    }

    try {
      await uploadUniverse(currentProject, universeName, uploadedUniverse);
      console.log("Universe saved successfully.");
      setUniverseAdded(true);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Axios Error: " + e.message);
        error("Axios Error: " + e.message);
      }
    }

    onClose();
  };

  return (
    <Modal
      id="universe-upload-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="bt-modal-titlebar">
          <label
            className="bt-modal-titlebar-title"
            htmlFor="actionName"
            style={{ textAlign: "center" }}
          >
            Upload a zip file with your universe
          </label>
          <CloseIcon
            className="bt-modal-titlebar-close bt-icon"
            onClick={() => {
              handleCancel();
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="bt-form-row">
          <div className="bt-modal-complex-input-row-container">
            <input
              className="bt-modal-complex-input"
              onChange={handleFileReader}
              type="file"
              accept=".zip,.rar,.7zip"
            />
          </div>
        </div>
        <div className="bt-upload-percentage">{uploadPercentage}</div>
        <div className="bt-form-row">
          <div className="bt-project-modal-creation-buttons-container">
            <div
              className="bt-roject-modal-create-button"
              onClick={() => {
                saveZipUniverse();
              }}
            >
              Accept
            </div>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default UniverseUploadModal;
