import React, { useState, useEffect, useRef, FormEventHandler } from "react";
import "./CreatePage.css";
import { ReactComponent as BackIcon } from "../../../Modal/img/back.svg";
import { ReactComponent as CloseIcon } from "../../../Modal/img/close.svg";
import axios from "axios";

const initialUniverseData = {
  universeName: "",
};

const CreatePage = ({
  setVisible,
  visible,
  onClose,
  currentProject,
  openError,
}: {
  setVisible: Function;
  visible: boolean;
  onClose: Function;
  currentProject: string;
  openError: Function;
}) => {
  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialUniverseData);
  const [availableUniverses, setUniversesDocker] = useState([]);

  const loadUniverseList = async () => {
    try {
      const listApiUrl = `/tree_api/list_docker_universes`;
      const response = await axios.get(listApiUrl);
      setUniversesDocker(response.data.universes_list);
    } catch (error) {
      console.error("Error while fetching universes list:", error);
      openError(`An error occurred while fetching the universes list`);
    }
  };

  useEffect(() => {
    if (visible && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }

    loadUniverseList();
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
    //TODO: create universe here
    onClose();
  };

  const handleFormSubmit = (data: any) => {};

  return (
    <>
      <div className="modal-titlebar">
        <BackIcon
          className="modal-titlebar-back icon"
          onClick={() => {
            setVisible(false);
          }}
          fill={"var(--icon)"}
        />
        <label
          className="modal-titlebar-title"
          htmlFor="actionName"
          style={{ textAlign: "center" }}
        >
          Create a Universe
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
        <div className="universe-create-name modal-complex-input-container">
          <input
            ref={focusInputRef}
            type="text"
            id="universeName"
            name="universeName"
            className="modal-complex-input"
            onChange={handleInputChange}
            autoComplete="off"
            maxLength={20}
            placeholder="Universe Name"
          />
          <label htmlFor="universeName" className="modal-complex-input-label">
            Universe Name
          </label>
          <label
            htmlFor="universeName"
            className="modal-complex-input-indications"
          >
            A unique name that is used for the universe folder and other
            resources. The name should be in lower case without spaces and
            should not start with a number. The maximum length is 20 characters.
          </label>
        </div>
      </div>
      <div className="modal-complex-input-row-container">
        <div id="create-new-universe" onClick={() => handleCreate()}>
          Create Universe
        </div>
      </div>
    </>
  );
};

export default CreatePage;
