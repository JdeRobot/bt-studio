import React, { useState, useEffect, useRef } from "react";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const initialNewFolderModalData = {
  renameData: "",
};

const RenameModal = ({
  onSubmit,
  isOpen,
  onClose,
  fileList,
  selectedEntry,
}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialNewFolderModalData);
  const [isCreationAllowed, allowCreation] = useState(false);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    if (isOpen && selectedEntry) {
      setFormState((prevFormData) => ({
        ...prevFormData,
        renameData: selectedEntry.name,
      }));
    }
    console.log(selectedEntry);
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    var isValidName = true;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));

    if (name === "renameData") {
      //TODO: improve check
      if (value !== "") {
        fileList.some((element) => {
          if (element.is_dir && element.name === value) {
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

  const getNewPath = (new_name) => {
    var split_path = selectedEntry.path.split("/"); // TODO: add for windows
    var parent_path = split_path.slice(0, split_path.length - 1).join("/");
    return parent_path + "/" + new_name;
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit(getNewPath(formState.renameData));
    setFormState(initialNewFolderModalData);
    allowCreation(false);
    onClose();
  };

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
    setFormState(initialNewFolderModalData);
    allowCreation(false);
  };

  if (!selectedEntry) {
    return null;
  }

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
            htmlFor="renameData"
            style={{ textAlign: "center" }}
          >
            Rename {selectedEntry.is_dir ? "Folder" : "File"}
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
              id="renameData"
              name="renameData"
              className={
                isCreationAllowed
                  ? "modal-complex-input"
                  : "modal-complex-input modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder={
                selectedEntry.is_dir ? "Rename Folder" : "Rename File"
              }
              required
            />
            <label htmlFor="renameData" className="modal-complex-input-label">
              Rename {selectedEntry.is_dir ? "Folder" : "File"}
            </label>
          </div>
        </div>
        <div className="form-row">
          <div className="button-row">
            <button type="reset">Cancel</button>
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

export default RenameModal;
