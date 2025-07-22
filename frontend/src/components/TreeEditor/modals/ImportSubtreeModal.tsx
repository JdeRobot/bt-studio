import React, { useState, useEffect, useRef, MutableRefObject } from "react";
import {
  Modal,
  ModalEditableList,
  ModalInputBox,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import {
  getLibraryTree,
  getSubtreeLibrary,
} from "../../../api_helper/TreeWrapper";
import { configureEngine } from "../../helper/TreeEditorHelper";
import createEngine, {
  CanvasWidget,
  DiagramModel,
} from "@projectstorm/react-diagrams";
import "./ImportSubtreeModal.css";

const initialData = {
  subTreeName: "",
};

const ImportSubtreeModal = ({
  onSubmit,
  isOpen,
  onClose,
  subTreeList,
}: {
  onSubmit: Function;
  isOpen: boolean;
  onClose: Function;
  subTreeList: string[];
}) => {
  const focusInputRef = useRef<any>(null);
  const [formState, setFormState] = useState(initialData);
  const [isCreationAllowed, allowCreation] = useState(false);
  const [availableSubtrees, setAvailableSubtree] = useState<any[]>([]);

  const getSubtrees = async () => {
    try {
      const response = await getSubtreeLibrary();
      var entry_list = [];
      for (const entry of response) {
        const graph_json = await getLibraryTree(entry);
        entry_list.push(<LibrarySubtree name={entry} tree={graph_json} />);
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

  useEffect(() => {
    getSubtrees();
  }, []);

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

    if (name === "subTreeName") {
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
    event.preventDefault();
    onSubmit(formState.subTreeName);
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
          isInputValid={isCreationAllowed || formState.subTreeName === ""}
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
          <div style={{width: "100%"}}>
            {Object.values(availableSubtrees).map((entry) => {
              return <>{entry}</>;
            })}
          </div>
          <div style={{width: "100%"}}>
            <ModalEditableList
              title="Standard library"
              list={["a","b", "c", "d","b", "c", "d","b", "c", "d","b", "c", "d","b", "c", "d"]}
              onSelect={function (event: any, entry: string): void {
                throw new Error("Function not implemented.");
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

const LibrarySubtree = ({ name, tree }: { name: string; tree: any }) => {
  const model = useRef(new DiagramModel());
  const engine = useRef(createEngine());
  const [fit, setFit] = useState(false);

  configureEngine(engine);

  // Deserialize and load the model
  model.current.deserializeModel(tree, engine.current);
  model.current.setLocked(true);
  engine.current.setModel(model.current);

  useEffect(() => {
    if (engine.current) {
      engine.current.zoomToFitNodes({
        margin: 5,
        nodes: model.current.getNodes(),
      });
      console.log("Fit");
    }
  }, [fit]);

  return (
    <div id={"subtree-" + name} style={{ margin: "1%", width: "30%" }}>
      <label>{name}</label>
      <CanvasWidget
        className={`subtree-library-canvas`}
        engine={engine.current}
      />
      <button
        onClick={() => {
          setFit(!fit);
        }}
      >
        Click
      </button>
    </div>
  );
};
