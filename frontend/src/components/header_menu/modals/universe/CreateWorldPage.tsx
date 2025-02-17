import React, { useState, useEffect, useRef } from "react";
import "./CreateWorldPage.css";
import { ReactComponent as BackIcon } from "../../../Modal/img/back.svg";
import { ReactComponent as CloseIcon } from "../../../Modal/img/close.svg";
import {
  createRoboticsBackendWorldAndRobot,
  listDockerWorlds,
  listDockerRobots,
} from "../../../../api_helper/TreeWrapper";
import { useError } from "../../../error_popup/ErrorModal";

const initialUniverseData = {
  universeName: "",
  dockerWorldName: "",
  dockerRobotName: "",
};

const CreateWorldPage = ({
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
  const dropdown = useRef<any>(null);
  const [formState, setFormState] = useState(initialUniverseData);
  const [availableWorlds, setWorldsDocker] = useState<string[]>([]);
  const [availableRobots, setRobotsDocker] = useState<string[]>([]);
  const [openDropdown, setOpenDropdown] = useState<boolean>(false);

  const loadWorldList = async () => {
    try {
      const response = await listDockerWorlds();
      setWorldsDocker(response);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching worlds list: " + e.message);
        error("Error while fetching worlds list: " + e.message);
      }
    }
  };

  const loadRobotList = async () => {
    try {
      const response = await listDockerRobots();
      setRobotsDocker(response);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching robots list: " + e.message);
        error("Error while fetching robots list: " + e.message);
      }
    }
  };

  useEffect(() => {
    if (visible && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    loadWorldList();
    loadRobotList();
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
    if (
      formState.universeName === "" &&
      formState.dockerWorldName === "" &&
      formState.dockerRobotName === ""
    ) {
      return;
    }

    if (
      !availableWorlds.includes(formState.dockerWorldName) ||
      !availableRobots.includes(formState.dockerRobotName)
    ) {
      //TODO: invalid docker universe
      return;
    }

    createRoboticsBackendWorldAndRobot(
      currentProject,
      formState.universeName,
      formState.dockerWorldName,
      formState.dockerRobotName,
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
      <div className="bt-modal-complex-input-row-container">
        <div className="bt-universe-create-name bt-modal-complex-input-container">
          <input
            ref={dropdown}
            type="text"
            id="dockerWorldName"
            name="dockerWorldName"
            list="dockerWorlds"
            className="bt-modal-complex-input"
            onChange={handleInputChange}
            placeholder="World Name"
            required={true}
          />
          <label
            htmlFor="dockerWorldName"
            className="bt-modal-complex-input-label"
          >
            Select Robotics Backend World
          </label>
          <datalist id="dockerWorlds">
            {availableWorlds &&
              availableWorlds.map((name) => <option value={name} />)}
          </datalist>
        </div>
      </div>
      <div className="bt-modal-complex-input-row-container">
        <div className="bt-universe-create-name bt-modal-complex-input-container">
          <input
            ref={dropdown}
            type="text"
            id="dockerRobotName"
            name="dockerRobotName"
            list="dockerRobots"
            className="bt-modal-complex-input"
            onChange={handleInputChange}
            placeholder="Robot Name"
            required={true}
          />
          <label
            htmlFor="dockerRobotName"
            className="bt-modal-complex-input-label"
          >
            Select Robotics Backend Robot
          </label>
          <datalist id="dockerRobots">
            {availableRobots &&
              availableRobots.map((name) => <option value={name} />)}
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

export default CreateWorldPage;
