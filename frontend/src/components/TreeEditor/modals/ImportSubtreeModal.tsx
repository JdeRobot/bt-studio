import React, { useState, useEffect, useRef, MutableRefObject } from "react";
import {
  Modal,
  ModalActionList,
  ModalEditableList,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import {
  getLibraryTree,
  getSubtreeLibrary,
  getUserLibraryTree,
  getUserSubtreeLibrary,
} from "../../../api_helper/TreeWrapper";
import { configureEngine } from "../../helper/TreeEditorHelper";
import createEngine, {
  CanvasWidget,
  DiagramModel,
} from "@projectstorm/react-diagrams";
import "./ImportSubtreeModal.css";

const initialData = {
  subtreeName: "",
};

const ImportSubtreeModal = ({
  project,
  onSubmit,
  isOpen,
  onClose,
  subTreeList,
}: {
  project: string;
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  subTreeList: string[];
}) => {
  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialData);
  const [isCreationAllowed, allowCreation] = useState(false);
  const [availableSubtrees, setAvailableSubtree] = useState<any[]>([]);
  const [availableUserSubtrees, setAvailableUserSubtree] = useState<any[]>([]);
  const [selectedSubtree, selectSubtree] = useState<{name: string, project?: string}|undefined>(undefined);

  const getSubtrees = async () => {
    try {
      const response = await getSubtreeLibrary();
      var entry_list = [];
      for (const entry of response) {
        const graph_json = await getLibraryTree(entry);
        const select = () => {selectSubtree({name: entry.tree, project: entry.project})}
        entry_list.push({
          name: entry,
          component: <LibrarySubtree name={entry} tree={graph_json} onSelect={select} />,
        });
      }
      setAvailableSubtree(entry_list);
      setFormState(initialData);
    } catch (e) {
      setAvailableSubtree([]);
      setFormState(initialData);
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        // error("Error while fetching project list: " + e.message);
      }
    }
  };

  const getUserSubtrees = async () => {
    try {
      const response = await getUserSubtreeLibrary(project);
      var entry_list = [];
      for (const entry of response) {
        const graph_json = await getUserLibraryTree(entry.project, entry.tree);
        const name = `${entry.project}: ${entry.tree}`
        const select = () => {selectSubtree({name: entry.tree, project: entry.project}); console.log({name: entry.tree, project: entry.project})}
        entry_list.push({
          name: name,
          component: <LibrarySubtree name={name} tree={graph_json} onSelect={select} />,
        });
      }
      setAvailableUserSubtree(entry_list);
    } catch (e) {
      setAvailableUserSubtree([]);
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        // error("Error while fetching project list: " + e.message);
      }
    }
  };

  useEffect(() => {
    getSubtrees();
  }, []);

  useEffect(() => {
    getUserSubtrees();
  }, [project]);

  useEffect(() => {
    console.log(selectSubtree)
  }, [selectSubtree]);

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    var isValidName = true;

    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));

    if (name === "subtreeName") {
      if (value !== "" && !value.includes(".")) {
        subTreeList.some((element: string) => {
          if (element === value) {
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

  const handleSubmit = (event: React.FormEvent<HTMLFormElement>) => {
    console.log("Selected",selectedSubtree)
    event.preventDefault();
    onSubmit(formState.subtreeName);
    setFormState(initialData);
    allowCreation(false);
    onClose();
  };

  const handleCancel = (event: React.FormEvent<HTMLFormElement> | null) => {
    if (event) {
      event.preventDefault();
    }
    onClose();
    setFormState(initialData);
    allowCreation(false);
  };

  return (
    <Modal
      id="import-subtree-modal"
      isOpen={isOpen}
      onClose={onClose}
      onSubmit={handleSubmit}
      onReset={handleCancel}
    >
      <ModalTitlebar
        title="Import subtree from library"
        htmlFor="subtreeName"
        hasClose
        handleClose={() => {
          handleCancel(null);
        }}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={isCreationAllowed || formState.subtreeName === ""}
          ref={focusInputRef}
          id="subtreeName"
          placeholder="Subtree Name"
          onChange={handleInputChange}
          type="text"
          autoComplete="off"
          required
        />
      </ModalRow>
      <ModalRow type="all">
        <div
          style={{
            display: "flex",
            flexDirection: "row",
            width: "100%",
            justifyContent: "space-around",
          }}
        >
          <div style={{ width: "100%" }}>
            <ModalActionList
              title="User library"
              list={availableUserSubtrees}
              onSelect={(event: any, entry: string) => {
                console.log(entry)
              }}
            />
          </div>
          <div style={{ width: "100%" }}>
            <ModalActionList
              title="Standard library"
              list={availableSubtrees}
              onSelect={(event: any, entry: string) => {
                console.log(entry)
              }}
            />
          </div>
        </div>
      </ModalRow>
      <ModalRow type="buttons">
        <button type="submit" id="import-subtree" disabled={!isCreationAllowed}>
          Import
        </button>
      </ModalRow>
    </Modal>
  );
};

export default ImportSubtreeModal;

const LibrarySubtree = ({ name, tree, onSelect }: { name: string; tree: any, onSelect: Function }) => {
  const model = useRef(new DiagramModel());
  const engine = useRef(
    createEngine({
      registerDefaultZoomCanvasAction: false,
      registerDefaultPanAndZoomCanvasAction: false,
    })
  );
  const [fit, setFit] = useState(false);

  configureEngine(engine);

  // Deserialize and load the model
  model.current.deserializeModel(tree, engine.current);
  model.current.setLocked(true);
  engine.current.setModel(model.current);

  useEffect(() => {
    if (engine.current) {
      const state: any = engine.current.getStateMachine().getCurrentState();
      state.dragCanvas.config.allowDrag = false;
      engine.current.zoomToFitNodes({ margin: 50 });
    }
  }, []);

  return (
    <div style={{ display: "flex", flexDirection: "column", width: "90%" }}>
      <ModalRow type="all">
        <CanvasWidget
          className={`subtree-library-canvas`}
          engine={engine.current}
        />
      </ModalRow>
      <ModalRow type="buttons">
        <button onClick={() => onSelect()} type="button" id="import-subtree">
          Select
        </button>
      </ModalRow>
    </div>
  );
};
