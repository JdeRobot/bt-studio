import React, { useState, useEffect, useRef } from "react";
import "./ErrorModal.css";
import Modal from "../Modal/Modal";

const ErrorModal = ({ onSubmit, isOpen, onClose }) => {
  const focusInputRef = useRef(null);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
  };

  return (
    <Modal
      id="error-modal"
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
          Warning
        </label>
      </div>
      <div className="form-row">
        <div className="error-modal-buttons-container">
          <label className="modal-error-label" id="errorMsg"></label>
        </div>
      </div>
      <div className="form-row">
        <div className="error-modal-buttons-container">
          <div className="error-modal-button" onClick={() => onClose()}>
            Close
          </div>
        </div>
      </div>
    </Modal>
  );
};

export default ErrorModal;
