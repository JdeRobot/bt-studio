import React, { useState, useEffect, useRef } from "react";
import {
  Modal,
  ModalActionList,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
  ModalRowDataText,
} from "jderobot-ide-interface";
import {
  getLibraryTree,
  getSubtreeLibrary,
  getUserLibraryTree,
  getUserSubtreeLibrary,
  importLibrarySubtree,
  importUserLibrarySubtree,
} from "../../../api_helper/TreeWrapper";
import { configureEngine, publish } from "../../helper/TreeEditorHelper";
import createEngine, { DiagramModel } from "@projectstorm/react-diagrams";
import "./ImportSubtreeModal.css";
import {
  StyledLibraryCanvas,
  StyledLibraryEntry,
} from "Styles/Modal/SubtreeLibrary.styles";

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
  const [selectedSubtree, selectSubtree] = useState<
    { name: string; project?: string } | undefined
  >(undefined);

  const getSubtrees = async () => {
    try {
      const response = await getSubtreeLibrary();
      const entry_list = [];
      for (const entry of response) {
        const entryData = await getLibraryTree(entry);

        entry_list.push({
          name: entry,
          base_name: entry,
          component: (
            <LibrarySubtree
              name={entry}
              tree={entryData.graph_json}
              btOrder={entryData.btOrder}
              actions={entryData.actions}
              subtrees={entryData.subtrees}
              onSelect={selectEntry}
            />
          ),
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

  const selectEntry = (name: string, project?: string) => {
    selectSubtree({ name, project });
  };

  const getUserSubtrees = async () => {
    try {
      const response = await getUserSubtreeLibrary(project);
      const entry_list = [];
      for (const entry of response) {
        const entryData = await getUserLibraryTree(entry.project, entry.tree);
        entry_list.push({
          name: `${entry.project}: ${entry.tree}`,
          component: (
            <LibrarySubtree
              name={entry.tree}
              tree={entryData.graph_json}
              btOrder={entryData.btOrder}
              actions={entryData.actions}
              subtrees={entryData.subtrees}
              project={entry.project}
              onSelect={selectEntry}
            />
          ),
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
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    let isValidName = true;

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

  const handleSubmit = async (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (selectedSubtree === undefined) {
      onSubmit();
      return;
    }

    // TODO: add modal for all the changes

    if (selectedSubtree.project === undefined) {
      // Import from standard library
      await importLibrarySubtree(
        project,
        selectedSubtree.name,
        formState.subtreeName,
      );
    } else {
      // Import from user library
      await importUserLibrarySubtree(
        project,
        selectedSubtree.name,
        selectedSubtree.project,
        formState.subtreeName,
      );
    }
    publish("updateSubtreeList");
    publish("updateExplorer-Code", { project: project });
    setFormState(initialData);
    allowCreation(false);
    onSubmit();
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
              selected={
                selectedSubtree
                  ? selectedSubtree.project
                    ? `${selectedSubtree.project}: ${selectedSubtree.name}`
                    : ""
                  : ""
              }
            />
          </div>
          <div style={{ width: "100%" }}>
            <ModalActionList
              title="Standard library"
              list={availableSubtrees}
              selected={selectedSubtree ? selectedSubtree.name : ""}
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

const LibrarySubtree = ({
  name,
  project,
  tree,
  btOrder,
  actions,
  subtrees,
  onSelect,
}: {
  name: string;
  project?: string;
  tree: any;
  btOrder: string;
  actions: string[];
  subtrees: string[];
  onSelect: Function;
}) => {
  const model = useRef(new DiagramModel());
  const engine = useRef(
    createEngine({
      registerDefaultZoomCanvasAction: false,
      registerDefaultPanAndZoomCanvasAction: false,
    }),
  );

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
    <StyledLibraryEntry>
      <ModalRow type="img">
        <StyledLibraryCanvas engine={engine.current} />
      </ModalRow>
      <ModalRowDataText title="Order" data={[btOrder]} />
      <ModalRowDataText title="Subtrees" data={subtrees} />
      <ModalRowDataText title="Actions" data={actions} />
      <ModalRow type="buttons">
        <button
          onClick={() => onSelect(name, project)}
          type="button"
          id="import-subtree"
          style={{ margin: "10px" }}
        >
          Select
        </button>
      </ModalRow>
    </StyledLibraryEntry>
  );
};
