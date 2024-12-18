import React, { useState, useEffect, useRef, FormEventHandler } from "react";
import "./CreatePage.css";
import { ReactComponent as BackIcon } from "../../../Modal/img/back.svg";
import { ReactComponent as CloseIcon } from "../../../Modal/img/close.svg";
import axios from "axios";
import { createRoboticsBackendUniverse } from "../../../../api_helper/TreeWrapper";

const initialUniverseData = {
  universeName: "",
  dockerUniverseName: "",
};

const CreatePage = ({
  setVisible,
  visible,
  onClose,
  currentProject,
  openError,
}: {
  setVisible: Function;
  visible: boolean;
  onClose: Function;
  currentProject: string;
  openError: Function;
}) => {
  const focusInputRef = useRef<any>(null);
  const dropdown = useRef<any>(null);
  const [formState, setFormState] = useState(initialUniverseData);
  const [availableUniverses, setUniversesDocker] = useState<string[]>([]);
  const [openDropdown, setOpenDropdown] = useState<boolean>(false);

  const loadUniverseList = async () => {
    try {
      const listApiUrl = `/bt_studio/list_docker_universes`;
      const response = await axios.get(listApiUrl);
      setUniversesDocker(response.data.universes);
    } catch (error) {
      console.error("Error while fetching universes list:", error);
      openError(`An error occurred while fetching the universes list`);
    }
  };

  useEffect(() => {
    if (visible && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    loadUniverseList();
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

  const handleCreate = () => {
    if (formState.universeName === "" && formState.dockerUniverseName === "") {
      return;
    }

    if (!availableUniverses.includes(formState.dockerUniverseName)) {
      //TODO: invalid docker universe
      return;
    }

    createRoboticsBackendUniverse(
      currentProject,
      formState.universeName,
      formState.dockerUniverseName,
    );
    setVisible(false);
  };

  const closeDropdown = (e: any) => {
    if (openDropdown && !dropdown.current?.contains(e.target)) {
      setOpenDropdown(false);
    }
  };

  document.addEventListener("mousedown", closeDropdown);

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
          <label htmlFor="universeName" className="bt-modal-complex-input-label">
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
      <div className="bt-modal-complex-input-row-container">
        <div className="bt-universe-create-name bt-modal-complex-input-container">
          <input
            ref={dropdown}
            type="text"
            id="dockerUniverseName"
            name="dockerUniverseName"
            list="dockerUniverses"
            className="bt-modal-complex-input"
            onChange={handleInputChange}
            placeholder="Universe Name"
            required={true}
          />
          <label
            htmlFor="dockerUniverseName"
            className="bt-modal-complex-input-label"
          >
            Select Robotics Backend Universe
          </label>
          <datalist id="dockerUniverses">
            {availableUniverses &&
              availableUniverses.map((name) => <option value={name} />)}
          </datalist>
        </div>
      </div>
      <div className="bt-modal-complex-input-row-container">
        <div id="create-new-universe" onClick={() => handleCreate()}>
          Create Universe
        </div>
      </div>
    </>
  );
};

export default CreatePage;
