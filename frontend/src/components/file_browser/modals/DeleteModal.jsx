import React, { useState, useEffect, useRef } from "react";
import "./DeleteModal.css";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

// TODO add a way to select if plain text file
const DeleteModal = ({ onSubmit, isOpen, onClose, selectedEntry }) => {
  //TODO: use relative path instead of absolute one

  useEffect(() => {
    // if (isOpen && focusInputRef.current) {
    //   setTimeout(() => {
    //     focusInputRef.current.focus();
    //   }, 0);
    // }
  }, [isOpen]);

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit();
  };

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
  };

  return (
    <Modal
      id="delete-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label
            className="modal-titlebar-title"
            style={{ textAlign: "center" }}
          >
            Delete confirmation
          </label>
          <CloseIcon
            className="modal-titlebar-close icon"
            onClick={() => {
              handleCancel();
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="form-row">
          <div className="delete-modal-prompt">
            <label> Do you want to delete {selectedEntry} ?</label>
          </div>
        </div>
        <div className="button-row">
          <button type="reset" id="cancel-delete-selected">
            Cancel
          </button>
          <button type="submit" id="delete-selected-button">
            Delete
          </button>
        </div>
      </form>
    </Modal>
  );
};

export default DeleteModal;
