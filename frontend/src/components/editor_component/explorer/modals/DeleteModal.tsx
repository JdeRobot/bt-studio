import { useEffect } from "react";
import "./DeleteModal.css";
import Modal from "../../../Modal/Modal";

import { ReactComponent as CloseIcon } from "../../../Modal/img/close.svg";
import { Entry } from "../Explorer";

// TODO add a way to select if plain text file
const DeleteModal = ({
  onSubmit,
  isOpen,
  onClose,
  selectedEntry,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  selectedEntry: Entry;
}) => {
  //TODO: use relative path instead of absolute one

  useEffect(() => {
    // if (isOpen && focusInputRef.current) {
    //   setTimeout(() => {
    //     focusInputRef.current.focus();
    //   }, 0);
    // }
  }, [isOpen]);

  const handleSubmit = (event: any) => {
    event.preventDefault();
    onSubmit();
  };

  const handleCancel = (event: any) => {
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
        <div className="bt-modal-titlebar">
          <label
            className="bt-modal-titlebar-title"
            style={{ textAlign: "center" }}
          >
            Delete confirmation
          </label>
          <CloseIcon
            className="bt-modal-titlebar-close bt-icon"
            onClick={() => {
              handleCancel(undefined);
            }}
            fill={"var(--icon)"}
          />
        </div>
        <div className="bt-form-row">
          <div className="bt-delete-modal-prompt">
            <label> Do you want to delete {selectedEntry.name} ?</label>
          </div>
        </div>
        <div className="bt-button-row">
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
