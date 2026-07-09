import React, { useState, useEffect, useRef } from "react";
import { createCustomWorld } from "BtApi/TreeWrapper";
import { ModalInputBox, ModalRow, ModalTitlebar } from "jderobot-ide-interface";

const initialWorldData = {
  worldName: "",
};

const CreateCustomPage = ({
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
  const focusInputRef = useRef<any>(null);
  const dropdown = useRef<any>(null);
  const [formState, setFormState] = useState(initialWorldData);
  const [openDropdown, setOpenDropdown] = useState<boolean>(false);

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

  const handleCreate = () => {
    if (formState.worldName === "") {
      return;
    }

    createCustomWorld(currentProject, formState.worldName);

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

export default CreateCustomPage;
