import React, { useState, useEffect, useRef } from "react";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const initialAddSubtreeModalData = {
  subTreeName: "",
};

const AddSubtreeModal = ({
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
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="bt-modal-titlebar">
          <label
            className="bt-modal-titlebar-title"
            htmlFor="folderName"
            style={{ textAlign: "center" }}
          >
            Create new subtree
          </label>
          <CloseIcon
            className="bt-modal-titlebar-close bt-icon"
            onClick={() => {
              handleCancel(null);
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="bt-modal-complex-input-row-container">
          <div className="bt-modal-complex-input-container">
            <input
              ref={focusInputRef}
              type="text"
              id="subTreeName"
              name="subTreeName"
              className={
                isCreationAllowed || formState.subTreeName === ""
                  ? "bt-modal-complex-input"
                  : "bt-modal-complex-input bt-modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="Subtree Name"
              required
            />
            <label
              htmlFor="subTreeName"
              className="bt-modal-complex-input-label"
            >
              Subtree Name
            </label>
          </div>
        </div>
        <div className="bt-form-row">
          <div className="bt-button-row">
            <button
              type="submit"
              id="create-new-action"
              disabled={!isCreationAllowed}
            >
              Create
            </button>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default AddSubtreeModal;
