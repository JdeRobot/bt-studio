import React, { useState, useEffect, useRef } from "react";
import "./UploadModal.css";
import Modal from "../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

// TODO add a way to select if plain text file
const UploadModal = ({ onSubmit, isOpen, onClose, selectedEntry }) => {
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
      id="upload-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label
            className="modal-titlebar-title"
            htmlFor="uploadName"
            style={{ textAlign: "center" }}
          >
            Upload
          </label>
          <CloseIcon
            className="modal-titlebar-close icon"
            onClick={() => {
              handleCancel();
            }}
            fill={"var(--icon)"}
          />
        </div>
      </form>
    </Modal>
  );
};

export default UploadModal;
