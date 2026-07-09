import React, { useState, useEffect, useRef } from "react";
import {
  createRoboticsBackendWorld,
  listDockerWorlds,
} from "BtApi/TreeWrapper";
import {
  ModalInputBox,
  ModalInputDropdown,
  ModalRow,
  ModalTitlebar,
  useError,
} from "jderobot-ide-interface";

const initialWorldData = {
  worldName: "",
  dockerWorldName: "",
};

const CreatePage = ({
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
  const dropdown = useRef<any>(null);
  const [formState, setFormState] = useState(initialWorldData);
  const [availableWorlds, setWorldsDocker] = useState<string[]>([]);
  const [openDropdown, setOpenDropdown] = useState<boolean>(false);

  const loadWorldList = async () => {
    try {
      const response = await listDockerWorlds();
      setWorldsDocker(response);
    } catch (e: unknown) {
      if (e instanceof Error) {
        error(e.message);
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
    if (formState.worldName === "" && formState.dockerWorldName === "") {
      return;
    }

    if (!availableWorlds.includes(formState.dockerWorldName)) {
      //TODO: invalid docker world
      return;
    }

    createRoboticsBackendWorld(
      currentProject,
      formState.worldName,
      formState.dockerWorldName,
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
      <ModalRow type="input">
        <ModalInputDropdown
          isInputValid={true}
          ref={dropdown}
          entries={availableWorlds}
          id="dockerWorldName"
          placeholder="Select Robotics Backend World"
          onChange={handleInputChange}
          type="text"
          required
        ></ModalInputDropdown>
      </ModalRow>
      <ModalRow type="buttons">
        <button
          type="button"
          id="create-new-world"
          onClick={() => handleCreate()}
          style={{ minWidth: "fit-content", padding: "0 1rem" }}
        >
          Create World
        </button>
      </ModalRow>
    </>
  );
};

export default CreatePage;
