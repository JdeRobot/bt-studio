import React, { useState, useEffect, useRef } from "react";
import "./NewActionModal.css";
import Modal from "../Modal/Modal";
import empty_template from "./img/empty_template.svg";
import action_template from "./img/action_template.svg";
import io_template from "./img/io_template.svg";
import close_modal_img from "../Modal/img/close.svg";

const initialNewActionModalData = {
  actionName: "",
  templateType: "empty",
  allowCreation: false,
};

function CreateButton({ hasToDisplay }) {
  if (hasToDisplay) {
    return (
      <button type="submit" id="create-new-action">
        Create
      </button>
    );
  }
  return null;
}

const NewActionModal = ({ onSubmit, isOpen, onClose, fileList }) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialNewActionModalData);
  const [template, setTemplate] = useState("empty");
  const [createButton, setcreateButton] = useState(false);

  const onOptionChange = (e) => {
    setTemplate(e.target.value);
    handleInputChange(e);
  };

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
    setTemplate("empty");
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    if (name === "actionName") {
      setcreateButton(
        value !== "" && !fileList.includes(value) && !value.includes("."),
      );
    }
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit(formState);
    setFormState(initialNewActionModalData);
    setcreateButton(false);
  };

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }
    onClose(formState);
    setFormState(initialNewActionModalData);
    setcreateButton(false);
  };

  return (
    <Modal
      id="new-action-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label
            className="modal-titlebar-title"
            htmlFor="actionName"
            style={{ textAlign: "center" }}
          >
            Create new action
          </label>
          <img
            className="modal-titlebar-close"
            onClick={() => {
              handleCancel();
            }}
            src={close_modal_img}
          ></img>
        </div>
        <div className="modal-complex-input-row-container">
          <div className="modal-complex-input-container">
            <input
              ref={focusInputRef}
              type="text"
              id="actionName"
              name="actionName"
              className="modal-complex-input"
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="Action Name"
              required
            />
            <label for="actionName" class="modal-complex-input-label">
              Action Name
            </label>
          </div>
        </div>
        <div className="form-row" id="templates-list">
          <label htmlFor="templateType" className="templates-list-title">
            Template
          </label>
          <div className="templates-list-container">
            <div className="templates-col">
              <label>
                <input
                  type="radio"
                  name="templateType"
                  value="empty"
                  id="emptyTemplate"
                  checked={template === "empty"}
                  onChange={onOptionChange}
                />
                <div htmlFor="emptyTemplate">
                  <img className="icon" src={empty_template}></img>
                  <p> Empty </p>
                </div>
              </label>
            </div>
            <div className="templates-col">
              <label>
                <input
                  type="radio"
                  name="templateType"
                  value="action"
                  id="actionTemplate"
                  checked={template === "action"}
                  onChange={onOptionChange}
                />
                <div htmlFor="actionTemplate">
                  <img className="icon" src={action_template}></img>
                  <p> Action </p>
                </div>
              </label>
            </div>
            <div className="templates-col">
              <label>
                <input
                  type="radio"
                  name="templateType"
                  value="io"
                  id="ioTemplate"
                  checked={template === "io"}
                  onChange={onOptionChange}
                />
                <div htmlFor="ioTemplate">
                  <img className="icon" src={io_template}></img>
                  <p> I/O </p>
                </div>
              </label>
            </div>
          </div>
        </div>
        <div className="form-row">
          <CreateButton hasToDisplay={createButton} />
        </div>
      </form>
    </Modal>
  );
};

export default NewActionModal;
