import React, { useRef, useEffect, useState } from "react";
import "./Modal.css";
import { ReactComponent as CloseIcon } from "./img/close.svg";

const Modal = ({
  id = "modal",
  isOpen,
  hasCloseBtn = true,
  onClose,
  children,
} : {
id: string;
isOpen: boolean;
hasCloseBtn: boolean;
onClose: Function;
children: any;
}) => {
  const [isModalOpen, setModalOpen] = useState<boolean>(isOpen);
  const modalRef = useRef<HTMLDialogElement>(null);

  const handleCloseModal = () => {
    if (onClose) {
      onClose();
    }
    setModalOpen(false);
  };

  const handleKeyDown = (event: any) => {
    if (event.key === "Escape") {
      handleCloseModal();
    }
  };

  useEffect(() => {
    setModalOpen(isOpen);
  }, [isOpen]);

  useEffect(() => {
    const modalElement = modalRef.current;

    document.getElementById(id)!.focus();
    if (modalElement) {
      if (isModalOpen) {
        modalElement.showModal();
      } else {
        modalElement.close();
      }
    }
  }, [isModalOpen]);

  return (
    <dialog
      id={id}
      ref={modalRef}
      onKeyDown={handleKeyDown}
      className="bt-modal"
    >
      <div className="bt-modal-contents">{children}</div>
    </dialog>
  );
};

export default Modal;

export const ModalTitlebar = ({ title, htmlFor, handleCancel } : { title:string, htmlFor:string, handleCancel: Function }) => {
  return (
    <div className="bt-modal-titlebar">
      <label
        className="bt-modal-titlebar-title"
        htmlFor={htmlFor}
        style={{ textAlign: "center" }}
      >
        {title}
      </label>
      <CloseIcon
        className="bt-modal-titlebar-close bt-icon"
        onClick={() => {
          handleCancel();
        }}
        fill={"var(--icon)"}
      />
    </div>
  );
};
