import React, { useState, useEffect, useRef } from "react";
import "./NewFileModal.css";
import Modal, { ModalTitlebar } from "../../Modal/Modal";

import { ReactComponent as EmptyTeplateIcon } from "../img/empty_template.svg";
import { ReactComponent as ActionTeplateIcon } from "../img/action_template.svg";
import { ReactComponent as IOTeplateIcon } from "../img/io_template.svg";
import { Entry } from "../FileBrowser";

export interface newFileModalData {
  fileType: string;
  fileName: string;
  templateType: string;
}

const initialNewFileModalData = {
  fileType: "plain",
  fileName: "",
  templateType: "empty",
};

class CardEntryProps {
  public value: string;
  public id: string;
  public icon: any;
  public text: string;

  constructor(value: string, id: string, icon: any, text: string) {
    this.value = value;
    this.id = id;
    this.icon = icon;
    this.text = text;
  }
}

///////////////////////// TYPES ////////////////////////////////////////////////
const plain = new CardEntryProps(
  "plain",
  "plainType",
  <ActionTeplateIcon className="bt-icon" fill={"var(--icon)"} />,
  "Plain File",
);
const actions = new CardEntryProps(
  "actions",
  "actionsType",
  <IOTeplateIcon className="bt-icon" fill={"var(--icon)"} />,
  "Action",
);

///////////////////////// ACTIONS //////////////////////////////////////////////
const empty = new CardEntryProps(
  "empty",
  "emptyTemplate",
  <EmptyTeplateIcon className="bt-icon" stroke={"var(--icon)"} />,
  "Empty",
);
const action = new CardEntryProps(
  "action",
  "actionTemplate",
  <ActionTeplateIcon className="bt-icon" fill={"var(--icon)"} />,
  "Action",
);
const io = new CardEntryProps(
  "io",
  "ioTemplate",
  <IOTeplateIcon className="bt-icon" fill={"var(--icon)"} />,
  "I/O",
);

const NewFileModal = ({
  onSubmit,
  isOpen,
  onClose,
  fileList,
  location,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  fileList: Entry[];
  location: string;
}) => {
  const focusInputRef = useRef<HTMLInputElement>(null);
  const [formState, setFormState] = useState(initialNewFileModalData);
  const [template, setTemplate] = useState<string>("empty");
  const [creationType, setCreationType] = useState<string>("plain");
  const [isCreationAllowed, allowCreation] = useState<boolean>(false);
  // Search lists for valid names
  const [searchActionsList, setSearchActionsList] = useState<Entry[]>([]);
  const [searchPlainList, setSearchPlainList] = useState<Entry[]>([]);

  const typesCardEntryProps = [plain, actions];
  const actionsCardEntryProps = [empty, action, io];

  const onOptionTypeChange = (e: any) => {
    setCreationType(e.target.value);
    handleInputChange(e);
  };

  const onOptionTemplateChange = (e: any) => {
    setTemplate(e.target.value);
    handleInputChange(e);
  };

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current!.focus();
      }, 0);
    }
    setCreationType("plain");
    setTemplate("empty");

    if (isOpen) {
      //NOTE: One for actions and one for location
      createValidNamesList(location, setSearchPlainList);
      createValidNamesList("actions", setSearchActionsList);
    }
  }, [isOpen]);

  useEffect(() => {
    updateCreation(formState["fileName"]);
  }, [creationType]);

  const createValidNamesList = (orig_path: string, callback: Function) => {
    let search_list = fileList;

    if (orig_path) {
      var path = orig_path.split("/");

      for (let index = 0; index < path.length; index++) {
        search_list = search_list.find(
          (entry) => entry.name === path[index] && entry.is_dir,
        )!.files;
      }
    }

    console.log(search_list);

    if (search_list) {
      callback(search_list);
    } else {
      callback([]);
    }
  };

  const handleInputChange = (event: any) => {
    const { name, value } = event.target;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));

    if (name === "fileName") {
      updateCreation(value);
    }
  };

  const updateCreation = (newName: string) => {
    var isValidName = true;
    var preCheck, checkList;

    if (creationType === "actions") {
      preCheck =
        newName !== "" && !newName.includes(".") && !newName.includes("/");
      checkList = searchActionsList;
    } else {
      preCheck = newName !== "" && !newName.includes("/");
      checkList = searchPlainList;
    }

    if (preCheck && checkList) {
      checkList.some((element) => {
        var name = element.name;

        if (creationType === "actions") {
          name = name.replace(".py", "");
        }

        if (name === newName) {
          isValidName = false;
          return true;
        }
        return false;
      });
    } else {
      isValidName = false;
    }
    console.log(creationType, checkList);

    allowCreation(isValidName);
  };

  const handleSubmit = (event: any) => {
    event.preventDefault();
    onSubmit(location, formState);
    setFormState(initialNewFileModalData);
    allowCreation(false);
  };

  const handleCancel = (event: any) => {
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
        <div className="bt-modal-complex-input-row-container">
          <div className="bt-modal-complex-input-container">
            <input
              ref={focusInputRef}
              type="text"
              id="fileName"
              name="fileName"
              className={
                isCreationAllowed || formState.fileName === ""
                  ? "bt-modal-complex-input"
                  : "bt-modal-complex-input bt-modal-complex-input-invalid"
              }
              onChange={handleInputChange}
              autoComplete="off"
              placeholder="File Name"
              required
            />
            <label htmlFor="fileName" className="bt-modal-complex-input-label">
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
        <div className="bt-form-row">
          <div className="bt-button-row">
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
}: {
  contentArray: CardEntryProps[];
  title: string;
  id: string;
  name: string;
  checkedVariable: string;
  checkedCallback: (e: any) => void;
}) => {
  return (
    <div className="bt-form-row" id={id}>
      <label htmlFor="templateType" className="bt-templates-list-title">
        {title}
      </label>
      <div className="bt-templates-list-container">
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
}: {
  cardEntryProp: CardEntryProps;
  name: string;
  checkedVariable: string;
  checkedCallback: (e: any) => void;
}) => {
  return (
    <div className="bt-templates-col">
      <label>
        <input
          type="radio"
          name={name}
          value={cardEntryProp.value}
          id={cardEntryProp.id}
          checked={checkedVariable === cardEntryProp.value}
          onChange={checkedCallback}
        />
        <div id={"button-" + cardEntryProp.id}>
          {cardEntryProp.icon}
          <p> {cardEntryProp.text} </p>
        </div>
      </label>
    </div>
  );
};
