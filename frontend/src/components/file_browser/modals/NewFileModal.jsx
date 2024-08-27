import React, { useState, useEffect, useRef } from "react";
import "./NewFileModal.css";
import Modal, { ModalTitlebar } from "../../Modal/Modal";
import { CardEntryProps } from "./NewFileTypes";

import { ReactComponent as EmptyTeplateIcon } from "../img/empty_template.svg";
import { ReactComponent as ActionTeplateIcon } from "../img/action_template.svg";
import { ReactComponent as IOTeplateIcon } from "../img/io_template.svg";

const initialNewFileModalData = {
  fileType: "plain",
  fileName: "",
  templateType: "empty",
};

///////////////////////// TYPES ////////////////////////////////////////////////
const plain = new CardEntryProps(
  "plain",
  "plainType",
  <ActionTeplateIcon className="icon" fill={"var(--icon)"} />,
  "Plain File"
);
const actions = new CardEntryProps(
  "actions",
  "actionsType",
  <IOTeplateIcon className="icon" fill={"var(--icon)"} />,
  "Action"
);

///////////////////////// ACTIONS //////////////////////////////////////////////
const empty = new CardEntryProps(
  "empty",
  "emptyTemplate",
  <EmptyTeplateIcon className="icon" stroke={"var(--icon)"} />,
  "Empty"
);
const action = new CardEntryProps(
  "action",
  "actionTemplate",
  <ActionTeplateIcon className="icon" fill={"var(--icon)"} />,
  "Action"
);
const io = new CardEntryProps(
  "io",
  "ioTemplate",
  <IOTeplateIcon className="icon" fill={"var(--icon)"} />,
  "I/O"
);

const NewFileModal = ({ onSubmit, isOpen, onClose, fileList, location }) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialNewFileModalData);
  const [template, setTemplate] = useState("empty");
  const [creationType, setCreationType] = useState("plain");
  const [isCreationAllowed, allowCreation] = useState(false);
  // Search lists for valid names
  const [searchActionsList, setSearchActionsList] = useState(null);
  const [searchPlainList, setSearchPlainList] = useState(null);

  const typesCardEntryProps = [plain, actions];
  const actionsCardEntryProps = [empty, action, io];

  const onOptionTypeChange = (e) => {
    setCreationType(e.target.value);
    handleInputChange(e);
  };

  const onOptionTemplateChange = (e) => {
    setTemplate(e.target.value);
    handleInputChange(e);
  };

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
    setCreationType("plain");
    setTemplate("empty");

    if (isOpen && location) {
      //TODO: one for actions and one for location
      createValidNamesList(location, setSearchPlainList);
      createValidNamesList("actions", setSearchActionsList);
    }
  }, [isOpen]);

  const createValidNamesList = (orig_path, callback) => {
    var path = orig_path.split("/");
    let search_list = fileList;

    for (let index = 0; index < path.length; index++) {
      search_list = search_list.find(
        (entry) => entry.name === path[index] && entry.is_dir
      ).files;
    }

    console.log(search_list);

    if (search_list) {
      callback(search_list);
    } else {
      callback([]);
    }
  };

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    var isValidName = true;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));

    if (name === "fileName") {
      var preCheck;
      var checkList;

      if (creationType === "actions") {
        preCheck = value !== "" && !value.includes(".") && !value.includes("/");
        checkList = searchActionsList;
      } else {
        preCheck = value !== "" && !value.includes("/");
        checkList = searchPlainList;
      }

      if (preCheck) {
        checkList.some((element) => {
          // TODO: if action remove .py
          if (element.name === value) {
            isValidName = false;
            return true;
          }
          return false;
        });
      } else {
        isValidName = false;
      }

      allowCreation(isValidName);
    }
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit(location, formState);
    setFormState(initialNewFileModalData);
    allowCreation(false);
  };

  const handleCancel = (event) => {
    if (event) {
      event.preventDefault();
    }

    onClose();
    setFormState(initialNewFileModalData);
    allowCreation(false);
  };

  return (
    <Modal
      id="new-action-modal"
      hasCloseBtn={true}
      isOpen={isOpen}
      onClose={handleCancel}
    >
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <ModalTitlebar
          title="Create new file"
          htmlFor="fileName"
          handleCancel={handleCancel}
        />
        <div className="modal-complex-input-row-container">
          <div className="modal-complex-input-container">
            <input
              ref={focusInputRef}
              type="text"
              id="fileName"
              name="fileName"
              className={
                isCreationAllowed || formState.fileName === ""
                  ? "modal-complex-input"
                  : "modal-complex-input modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="File Name"
              required
            />
            <label htmlFor="fileName" className="modal-complex-input-label">
              File Name
            </label>
          </div>
        </div>
        <CardSelector
          contentArray={typesCardEntryProps}
          title="Select File Type"
          id="types-list"
          name="fileType"
          checkedVariable={creationType}
          checkedCallback={onOptionTypeChange}
        />
        {creationType === "actions" && (
          <CardSelector
            contentArray={actionsCardEntryProps}
            title="Select Template Type"
            id="templates-list"
            name="templateType"
            checkedVariable={template}
            checkedCallback={onOptionTemplateChange}
          />
        )}
        <div className="form-row">
          <div className="button-row">
            <button
              type="submit"
              id="create-new-action"
              disabled={!isCreationAllowed}
            >
              Create
            </button>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default NewFileModal;

const CardSelector = ({
  contentArray,
  title,
  id,
  name,
  checkedVariable,
  checkedCallback,
}) => {
  return (
    <div className="form-row" id={id}>
      <label htmlFor="templateType" className="templates-list-title">
        {title}
      </label>
      <div className="templates-list-container">
        {contentArray.map((x) => (
          <CardEntry
            cardEntryProp={x}
            name={name}
            checkedVariable={checkedVariable}
            checkedCallback={checkedCallback}
          />
        ))}
      </div>
    </div>
  );
};

const CardEntry = ({
  cardEntryProp,
  name,
  checkedVariable,
  checkedCallback,
}) => {
  return (
    <div className="templates-col">
      <label>
        <input
          type="radio"
          name={name}
          value={cardEntryProp.value}
          id={cardEntryProp.id}
          checked={checkedVariable === cardEntryProp.value}
          onChange={checkedCallback}
        />
        <div htmlFor="ioTemplate">
          {cardEntryProp.icon}
          <p> {cardEntryProp.text} </p>
        </div>
      </label>
    </div>
  );
};
