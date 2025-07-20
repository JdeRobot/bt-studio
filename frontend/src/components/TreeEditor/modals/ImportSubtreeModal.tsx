import React, { useState, useEffect, useRef } from "react";
import {
  Modal,
  ModalEditableList,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import { getSubtreeLibrary } from "../../../api_helper/TreeWrapper";

const initialData = {
  subTreeName: "",
};

const ImportSubtreeModal = ({
  onSubmit,
  isOpen,
  onClose,
  subTreeList,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  subTreeList: string[];
}) => {
  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialData);
  const [isCreationAllowed, allowCreation] = useState(false);
  const [availableSubtrees, setAvailableSubtree] = useState<any[]>([]);

  const getSubtrees = async () => {
    try {
      const response = await getSubtreeLibrary();
      setAvailableSubtree(response);
      setFormState(initialData);
    } catch (e) {
      setAvailableSubtree([]);
      setFormState(initialData);
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        // error("Error while fetching project list: " + e.message);
      }
    }
  };

  useEffect(() => {
    getSubtrees();
  }, []);

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
    onSubmit(formState.subTreeName);
    setFormState(initialData);
    allowCreation(false);
    onClose();
  };

  const handleCancel = (event: React.FormEvent<HTMLFormElement> | null) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
    setFormState(initialData);
    allowCreation(false);
  };

  return (
    <Modal
      id="import-subtree-modal"
      isOpen={isOpen}
      onClose={onClose}
      onSubmit={handleSubmit}
      onReset={handleCancel}
    >
      <ModalTitlebar
        title="Import subtree from library"
        htmlFor="subtreeName"
        hasClose
        handleClose={() => {
          handleCancel(null);
        }}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={isCreationAllowed || formState.subTreeName === ""}
          ref={focusInputRef}
          id="subtreeName"
          placeholder="Subtree Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
      <ModalRow type="all">
        <ModalEditableList
          list={Object.values(availableSubtrees)}
          onSelect={(e: any, entry: string) => {
            onClose(entry);
          }}
        />
      </ModalRow>
      <ModalRow type="buttons">
        <button
          type="submit"
          id="import-subtree"
          disabled={!isCreationAllowed}
        >
          Import
        </button>
      </ModalRow>
    </Modal>
  );
};

export default ImportSubtreeModal;
