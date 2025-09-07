import { useState, useEffect, useRef } from "react";
import {
  Modal,
  ModalInputBox,
  ModalInputSelectIcon,
  ModalRow,
  ModalTitlebar,
  Entry,
  ModalInputSelectIconEntry,
} from "jderobot-ide-interface";
import { ActionTeplateIcon, EmptyTeplateIcon, IOTeplateIcon } from "../icons";

export interface newFileData {
  fileType: string;
  fileName: string;
  templateType: string;
}

const initialNewFileModalData: newFileData = {
  fileType: "plain",
  fileName: "",
  templateType: "empty",
};

const CreateAction = ({
  onSubmit,
  isOpen,
  onClose,
  fileList,
  location,
  project,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  fileList: Entry[];
  location: string;
  project: string;
}) => {
  const focusInputRef = useRef<HTMLInputElement>(null);
  const [formState, setFormState] = useState(initialNewFileModalData);
  const [template, setTemplate] = useState<string>("empty");
  const [creationType, setCreationType] = useState<string>("plain");
  const [isCreationAllowed, allowCreation] = useState<boolean>(false);
  // Search lists for valid names
  const [searchActionsList, setSearchActionsList] = useState<Entry[]>([]);
  const [searchPlainList, setSearchPlainList] = useState<Entry[]>([]);

  ///////////////////////// TYPES ////////////////////////////////////////////////
  const plain: ModalInputSelectIconEntry = {
    id: "plain",
    title: "Plain File",
    iconType: "fill",
    icon: <ActionTeplateIcon viewBox="0 0 6.4 6.4" />,
  };

  const actions: ModalInputSelectIconEntry = {
    id: "actions",
    title: "Action",
    iconType: "fill",
    icon: <IOTeplateIcon viewBox="0 0 20 20" />,
  };

  ///////////////////////// ACTIONS //////////////////////////////////////////////
  const empty: ModalInputSelectIconEntry = {
    id: "empty",
    title: "Empty",
    iconType: "stroke",
    icon: <EmptyTeplateIcon viewBox="0 0 20 20" />,
  };

  const action: ModalInputSelectIconEntry = {
    id: "action",
    title: "Action",
    iconType: "fill",
    icon: <ActionTeplateIcon viewBox="0 0 6.4 6.4" />,
  };

  const io: ModalInputSelectIconEntry = {
    id: "io",
    title: "I/O",
    iconType: "fill",
    icon: <IOTeplateIcon viewBox="0 0 20 20" />,
  };

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
        const find = search_list.find(
          (entry) => entry.name === path[index] && entry.is_dir,
        );

        if (find !== undefined) {
          search_list = find.files;
        } else {
          search_list = [];
        }
      }
    }

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
    console.log(isValidName);
    allowCreation(isValidName);
  };

  const handleSubmit = (event: any) => {
    event.preventDefault();
    onSubmit(project, location, formState);
    setFormState(initialNewFileModalData);
    allowCreation(false);
    onClose();
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
      isOpen={isOpen}
      onClose={handleCancel}
      onSubmit={handleSubmit}
      onReset={handleCancel}
    >
      <ModalTitlebar
        title="Create new file"
        htmlFor="fileName"
        hasClose
        handleClose={handleCancel}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={isCreationAllowed || formState.fileName === ""}
          ref={focusInputRef as any}
          id="fileName"
          placeholder="File Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
      <ModalRow>
        <ModalInputSelectIcon
          id="fileType"
          title="Select File Type"
          onChange={onOptionTypeChange}
          selected={creationType}
          entries={[plain, actions]}
        />
      </ModalRow>
      {creationType === "actions" && (
        <ModalRow>
          <ModalInputSelectIcon
            id="templateType"
            title="Select Template Type"
            onChange={onOptionTemplateChange}
            selected={template}
            entries={[empty, action, io]}
          />
        </ModalRow>
      )}
      <ModalRow type="buttons">
        <button
          type="submit"
          id="create-new-action"
          disabled={!isCreationAllowed}
        >
          Create
        </button>
      </ModalRow>
    </Modal>
  );
};

export default CreateAction;
