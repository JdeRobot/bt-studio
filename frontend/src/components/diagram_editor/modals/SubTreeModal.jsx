import React, { useState, useEffect, useRef } from "react";
import Modal from "../../Modal/Modal";
import back_modal_img from "../../Modal/img/back.svg";
import close_modal_img from "../../Modal/img/close.svg";
import MinimalDiagramEditor from "../MinimalDiagramEditor";
import "./SubTreeModal.css";

const SubtreeModal = ({
  isOpen,
  onClose,
  initialJson,
  setResultJson,
  projectName,
  setDiagramEdited,
}) => {
  const handleCancel = () => {
    onClose();
  };

  return (
    <Modal id="sub-tree-modal" hasCloseBtn={true} isOpen={isOpen}>
      <form onReset={handleCancel}>
        <div className="modal-titlebar">
          <label
            className="modal-titlebar-title"
            htmlFor="actionName"
            style={{ textAlign: "center" }}
          >
            Sub Tree Editor
          </label>
          <img
            className="modal-titlebar-close"
            onClick={() => {
              handleCancel();
            }}
            src={close_modal_img}
          ></img>
        </div>
        <div>
          <MinimalDiagramEditor
            modelJson={initialJson}
            setResultJson={setResultJson}
            projectName={projectName}
            setDiagramEdited={setDiagramEdited}
            hasSubtrees={false}
          />
        </div>
      </form>
    </Modal>
  );
};

export default SubtreeModal;
