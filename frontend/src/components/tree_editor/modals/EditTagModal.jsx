import React, { useState, useEffect, useRef } from "react";

import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const initialEditTagModalData = {
  tagName: "",
};

const EditTagModal = ({ isOpen, onClose, currentActionNode }) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialEditTagModalData);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    currentActionNode.setName(value);
  };

  useEffect(() => {
    setFormState(initialEditTagModalData);
    document.getElementById("node-editor-modal").focus();
    if (currentActionNode) {
      document.getElementById("tagName").value = currentActionNode.getName();
    }
  }, [isOpen]);

  const horizontalScrolling = (e) => {
    e.preventDefault();
    var containerScrollPosition = e.target.scrollLeft;
    e.target.scrollBy({
      top: 0,
      left: e.deltaY,
      behaviour: "smooth",
    });
  };

  return (
    <Modal
      id="tag-editor-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <div className="modal-titlebar">
        <label
          className="modal-titlebar-title"
          htmlFor="actionName"
          style={{ textAlign: "center" }}
        >
          Edit port value
        </label>
        <CloseIcon
          className="modal-titlebar-close icon"
          onClick={() => {
            onClose();
          }}
          fill={"var(--icon)"}
        />
      </div>
      <div className="modal-complex-input-row-container">
        <div className="modal-complex-input-container">
          <input
            ref={focusInputRef}
            type="text"
            id="tagName"
            name="tagName"
            className="modal-complex-input"
            onChange={handleInputChange}
            autoComplete="off"
            placeholder="Tag Name"
            required
          />
          <label htmlFor="tagName" className="modal-complex-input-label">
            Tag Name
          </label>
        </div>
      </div>
    </Modal>
  );
};

export default EditTagModal;
