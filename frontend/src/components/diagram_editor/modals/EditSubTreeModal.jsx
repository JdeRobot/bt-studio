import React, { useState, useEffect, useRef } from "react";

import Modal from "../../Modal/Modal";
import DiagramEditor from "../DiagramEditor";

import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

const EditSubTreeModal = ({
  isOpen,
  onClose,
  currentProjectname,
  setModelJson,
  setProjectChanges,
  gazeboEnabled,
  manager,
  actionNodesData,
  btOrder,
  openError,
  setDiagramEditorReady,
}) => {
  const focusInputRef = useRef(null);

  useEffect(() => {
    console.log("EditSubTree modal open");
  }, [isOpen]);

  return (
    <Modal
      id="subtree-editor-modal"
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
      <div>Hello there</div>
    </Modal>
  );
};

export default EditSubTreeModal;
