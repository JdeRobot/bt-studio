import React, { useState, useEffect, useRef } from "react";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const initialNewFolderModalData = {
  folderName: "",
};

const NewFolderModal = ({ onSubmit, isOpen, onClose, fileList, location }) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialNewFolderModalData);
  const [isCreationAllowed, allowCreation] = useState(false);
  const [searchList, setSearchList] = useState(null);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    if (isOpen && location) {
      var path = location.split("/");

      let search_list = fileList;

      for (let index = 0; index < path.length; index++) {
        search_list = search_list.find(
          (entry) => entry.name === path[index] && entry.is_dir
        ).files;
      }

      if (search_list) {
        setSearchList(search_list);
      } else {
        setSearchList([]);
      }
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
        searchList.some((element) => {
          if (element.name === value) {
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

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit(location, formState.folderName);
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
                isCreationAllowed || formState.folderName === ""
                  ? "modal-complex-input"
                  : "modal-complex-input modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="Folder Name"
              required
            />
            <label htmlFor="folderName" className="modal-complex-input-label">
              Folder Name
            </label>
          </div>
        </div>
        <div className="form-row">
          <div className="button-row">
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

export default NewFolderModal;
