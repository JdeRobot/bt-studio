import React from "react";
import { useEffect, useRef } from "react";
import { DiagramEngine } from "@projectstorm/react-diagrams";

import {
  Modal,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";

import { TagNodeModel } from "../nodes/tag_node/TagNodeModel";

const EditTagModal = ({
  setFileContent,
  isOpen,
  onClose,
  currentActionNode,
  engine,
}: {
  setFileContent: Function;
  isOpen: boolean;
  onClose: Function;
  currentActionNode: TagNodeModel;
  engine: DiagramEngine;
}) => {
  const focusInputRef = useRef<any>(null);

  const handleInputChange = (event: any) => {
    const { name, value } = event.target;
    currentActionNode.setName(value);

    setFileContent();
    engine.repaintCanvas();
  };

  useEffect(() => {
    document.getElementById("tag-editor-modal")!.focus();
    if (currentActionNode) {
      const tagName: HTMLInputElement | null = document.getElementById(
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
    <Modal id="tag-editor-modal" isOpen={isOpen} onClose={onClose}>
      <ModalTitlebar
        title="Edit port value"
        htmlFor="actionName"
        hasClose
        handleClose={() => {
          onClose();
        }}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={true}
          ref={focusInputRef}
          id="tagName"
          placeholder="Tag Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
    </Modal>
  );
};

export default EditTagModal;
