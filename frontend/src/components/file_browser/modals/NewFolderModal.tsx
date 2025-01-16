import React, { useState, useEffect, useRef } from "react";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const initialNewFolderModalData = {
  folderName: "",
};

interface Entry {
  name: string;
  is_dir: boolean;
  path: string;
  files: Entry[];
}

const NewFolderModal = ({
  onSubmit,
  isOpen,
  onClose,
  fileList,
  location,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  fileList: Entry[];
  location: string;
}) => {
  const focusInputRef = useRef<HTMLInputElement>(null);
  const [formState, setFormState] = useState(initialNewFolderModalData);
  const [isCreationAllowed, allowCreation] = useState(false);
  const [searchList, setSearchList] = useState<Entry[]>([]);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current!.focus();
      }, 0);
    }

    if (isOpen) {
      let search_list = fileList;

      if (location) {
        var path = location.split("/");

        for (let index = 0; index < path.length; index++) {
          search_list = search_list.find(
            (entry: Entry) => entry.name === path[index] && entry.is_dir,
          )!.files;
        }
      }

      if (search_list) {
        setSearchList(search_list);
      } else {
        setSearchList([]);
      }
    }
  }, [isOpen]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
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

  const handleSubmit = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    onSubmit(location, formState.folderName);
    setFormState(initialNewFolderModalData);
    allowCreation(false);
    onClose();
  };

  const handleCancel = (event: React.FormEvent<HTMLFormElement> | null) => {
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
        <div className="bt-modal-titlebar">
          <label
            className="bt-modal-titlebar-title"
            htmlFor="folderName"
            style={{ textAlign: "center" }}
          >
            Create new folder
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
              id="folderName"
              name="folderName"
              className={
                isCreationAllowed || formState.folderName === ""
                  ? "bt-modal-complex-input"
                  : "bt-modal-complex-input bt-modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="Folder Name"
              required
            />
            <label
              htmlFor="folderName"
              className="bt-modal-complex-input-label"
            >
              Folder Name
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

export default NewFolderModal;
