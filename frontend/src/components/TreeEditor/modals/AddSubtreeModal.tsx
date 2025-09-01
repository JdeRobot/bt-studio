import React, { useState, useEffect, useRef } from "react";
import {
  Modal,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import { publish } from "../../helper/TreeEditorHelper";

const initialAddSubtreeModalData = {
  subTreeName: "",
};

const AddSubtreeModal = ({
  project,
  onSubmit,
  isOpen,
  onClose,
  subTreeList,
}: {
  project: string;
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  subTreeList: string[];
}) => {
  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialAddSubtreeModalData);
  const [isCreationAllowed, allowCreation] = useState(false);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    var isValidName = true;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));

    if (name === "subTreeName") {
      if (value !== "" && !value.includes(".")) {
        subTreeList.some((element: string) => {
          if (element === value) {
            isValidName = false;
            return true;
          }
          return false;
        });
      } else {
        isValidName = false;
      }

      allowCreation(isValidName);
    }
  };

  const handleSubmit = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    publish("updateSubtreeList");
    publish("updateExplorer-Code", { project: project });
    onSubmit(formState.subTreeName);
    setFormState(initialAddSubtreeModalData);
    allowCreation(false);
    onClose();
  };

  const handleCancel = (event: React.FormEvent<HTMLFormElement> | null) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
    setFormState(initialAddSubtreeModalData);
    allowCreation(false);
  };

  return (
    <Modal
      id="new-subtree-modal"
      isOpen={isOpen}
      onClose={onClose}
      onSubmit={handleSubmit}
      onReset={handleCancel}
    >
      <ModalTitlebar
        title="Create new subtree"
        htmlFor="folderName"
        hasClose
        handleClose={() => {
          handleCancel(null);
        }}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={isCreationAllowed || formState.subTreeName === ""}
          ref={focusInputRef}
          id="subTreeName"
          placeholder="Subtree Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
      <ModalRow type="buttons">
        <button
          type="submit"
          id="create-new-subtree"
          disabled={!isCreationAllowed}
        >
          Create
        </button>
      </ModalRow>
    </Modal>
  );
};

export default AddSubtreeModal;
