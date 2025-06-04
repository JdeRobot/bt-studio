import { useState, useEffect, useRef } from "react";
import { DiagramEngine, DiagramModel } from "@projectstorm/react-diagrams";

import Modal from "../../Modal/Modal";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

import { TagNodeModel } from "../nodes/tag_node/TagNodeModel";

const initialEditTagModalData = {
  tagName: "",
};

const EditTagModal = ({
  setFileContent,
  isOpen,
  onClose,
  currentActionNode,
  model,
  engine,
}: {
  setFileContent: Function;
  isOpen: boolean;
  onClose: Function;
  currentActionNode: TagNodeModel;
  model: any;
  // model: DiagramModel;
  engine: DiagramEngine;
}) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialEditTagModalData);

  const updateJsonState = () => {
    setFileContent(model.serialize());
  };

  const handleInputChange = (event: any) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    currentActionNode.setName(value);

    updateJsonState();
    engine.repaintCanvas();
  };

  useEffect(() => {
    setFormState(initialEditTagModalData);
    document.getElementById("tag-editor-modal")!.focus();
    if (currentActionNode) {
      var tagName: HTMLInputElement | null = document.getElementById(
        "tagName",
      ) as HTMLInputElement;
      if (tagName) {
        tagName.value = currentActionNode.getName();
      }
    }
  }, [isOpen]);

  // const horizontalScrolling = (e: any) => {
  //   e.preventDefault();
  //   var containerScrollPosition = e.target.scrollLeft;
  //   e.target.scrollBy({
  //     top: 0,
  //     left: e.deltaY,
  //     behaviour: "smooth",
  //   });
  // };

  return (
    <Modal
      id="tag-editor-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={onClose}
    >
      <div className="bt-modal-titlebar">
        <label
          className="bt-modal-titlebar-title"
          htmlFor="actionName"
          style={{ textAlign: "center" }}
        >
          Edit port value
        </label>
        <CloseIcon
          className="bt-modal-titlebar-close icon"
          onClick={() => {
            onClose();
          }}
          fill={"var(--icon)"}
        />
      </div>
      <div className="bt-modal-complex-input-row-container">
        <div className="bt-modal-complex-input-container">
          <input
            ref={focusInputRef}
            type="text"
            id="tagName"
            name="tagName"
            className="bt-modal-complex-input"
            onChange={handleInputChange}
            autoComplete="off"
            placeholder="Tag Name"
            required
          />
          <label htmlFor="tagName" className="bt-modal-complex-input-label">
            Tag Name
          </label>
        </div>
      </div>
    </Modal>
  );
};

export default EditTagModal;
