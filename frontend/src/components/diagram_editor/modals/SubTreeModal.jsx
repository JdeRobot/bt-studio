import React, { useState, useEffect, useRef } from "react";
import Modal from "../../Modal/Modal";
import back_modal_img from "../../Modal/img/back.svg";
import close_modal_img from "../../Modal/img/close.svg";
import DiagramEditor from "../DiagramEditor";
import "./SubTreeModal.css";
import { getSubtree } from "../../../api_helper/TreeWrapper";

const SubtreeModal = ({
  isOpen,
  onClose,
  projectName,
  subtreeName,
  setDiagramEdited,
}) => {
  // STATE
  const [initialJson, setInitialJson] = useState(null);
  const [resultJson, setResultJson] = useState(null);

  // EFFECTS
  useEffect(() => {
    const fetchSubtree = async () => {
      if (isOpen) {
        try {
          const response = await getSubtree(subtreeName, projectName);
          setInitialJson(JSON.parse(response));
        } catch (error) {
          console.error("Failed to fetch subtree:", error);
        }
      }
    };
    fetchSubtree();
  }, [isOpen, projectName, subtreeName]);

  const handleCancel = () => {
    onClose();
  };

  return (
    <Modal id="sub-tree-modal" hasCloseBtn={true} isOpen={isOpen}>
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
        {initialJson && (
          <DiagramEditor
            modelJson={initialJson}
            setResultJson={setResultJson}
            projectName={projectName}
            setDiagramEdited={setDiagramEdited}
            hasSubtrees={false}
          />
        )}
      </div>
    </Modal>
  );
};

export default SubtreeModal;
