import React, { useState, useEffect, useRef } from "react";
import { createCustomUniverse } from "../../../api_helper/TreeWrapper";
import {
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
  useError,
} from "jderobot-ide-interface";

const initialUniverseData = {
  universeName: "",
};

const CreateCustomPage = ({
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
    if (formState.universeName === "") {
      return;
    }

    createCustomUniverse(currentProject, formState.universeName);

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

export default CreateCustomPage;
