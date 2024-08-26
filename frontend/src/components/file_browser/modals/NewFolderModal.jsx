import React, { useState, useEffect, useRef } from "react";
import "./NewFolderModal.css";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const initialNewFolderModalData = {
  folderName: "",
  allowCreation: false,
};

function CreateButton({ hasToDisplay }) {
  if (hasToDisplay) {
    return (
      <button type="submit" id="create-new-folder">
        Create
      </button>
    );
  }
  return null;
}

const NewFolderModal = ({ onSubmit, isOpen, onClose, fileList, location }) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialNewFolderModalData);
  const [createButton, setcreateButton] = useState(false);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    var isValidName = true;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));

    if (name === "folderName") {
      if (value !== "" && !value.includes(".")) {
        fileList.some((element) => {
          if (element.is_dir && element.name === value) {
            isValidName = false;
            return true;
          }
        });
      } else {
        isValidName = false;
      }

      setcreateButton(isValidName);
    }
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit(location, formState.folderName);
    setFormState(initialNewFolderModalData);
    setcreateButton(false);
    onClose();
  };

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
    setFormState(initialNewFolderModalData);
    setcreateButton(false);
  };

  return (
    <Modal
      id="new-folder-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label
            className="modal-titlebar-title"
            htmlFor="folderName"
            style={{ textAlign: "center" }}
          >
            Create new folder
          </label>
          <CloseIcon
            className="modal-titlebar-close icon"
            onClick={() => {
              handleCancel();
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="modal-complex-input-row-container">
          <div className="modal-complex-input-container">
            <input
              ref={focusInputRef}
              type="text"
              id="folderName"
              name="folderName"
              className={
                createButton
                  ? "modal-complex-input"
                  : "modal-complex-input modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="Folder Name"
              required
            />
            <label for="folderName" class="modal-complex-input-label">
              Folder Name
            </label>
          </div>
        </div>
        <div className="form-row">
          <CreateButton hasToDisplay={createButton} />
        </div>
      </form>
    </Modal>
  );
};

export default NewFolderModal;
