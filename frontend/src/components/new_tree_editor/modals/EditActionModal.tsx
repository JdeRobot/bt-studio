import React, { useState, useEffect, useRef } from "react";
import { DiagramEngine, DiagramModel } from "@projectstorm/react-diagrams";
import "react-color-palette/css";
import { Saturation, Hue, ColorService, IColor } from "react-color-palette";

import {
  addPort,
  removePort,
  ActionNodePortType,
  changeColorNode,
  getActionFrame,
} from "../../helper/TreeEditorHelper";
import { rgbToLuminance } from "../../helper/colorHelper";

import { ReactComponent as AddIcon } from "../img/add.svg";
import { ReactComponent as DeleteIcon } from "../img/delete.svg";
import { ReactComponent as CancelIcon } from "../img/cancel.svg";
import { ReactComponent as AcceptIcon } from "../img/accept.svg";
import { ReactComponent as CloseIcon } from "../../Modal/img/close.svg";

import "./EditActionModal.css";
import Modal from "../../Modal/Modal";
import { BasicNodeModel } from "../../tree_editor/nodes/basic_node/BasicNodeModel";
import { OutputPortModel } from "../../tree_editor/nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "../../tree_editor/nodes/basic_node/ports/input_port/InputPortModel";

const initialEditActionModalData = {
  newInputName: "",
  newOutputName: "",
};

const EditActionModal = ({
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
  currentActionNode: BasicNodeModel;
  model: DiagramModel;
  engine: DiagramEngine;
}) => {
  const focusInputRef = useRef(null);
  const [color, setColor] = useState<IColor | undefined>(undefined);
  const [inputName, setInputName] = React.useState(false);
  const [outputName, setOutputName] = React.useState(false);
  const [allowCreation, setAllowCreation] = React.useState(false);
  const [formState, setFormState] = useState(initialEditActionModalData);
  const [update, setUpdate] = React.useState(false);

  const updateJsonState = () => {
    setFileContent(JSON.stringify(model.serialize()));
  };

  const handleInputChange = (event: any) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    setAllowCreation(
      (name === "newInputName" && isInputNameValid(value)) ||
        (name === "newOutputName" && isOutputNameValid(value))
    );
  };

  useEffect(() => {
    setInputName(false);
    setOutputName(false);
    setFormState(initialEditActionModalData);
    document.getElementById("node-editor-modal")!.focus();
    if (currentActionNode) {
      var rgb: IColor["rgb"] = ColorService.toRgb(currentActionNode.getColor());

      const newColor: IColor = {
        hex: ColorService.rgb2hex(rgb),
        rgb: rgb,
        hsv: ColorService.rgb2hsv(rgb),
      };

      setColor(newColor);
    }
  }, [isOpen]);

  useEffect(() => {
    if (update) {
      setUpdate(false);
    }
  }, [update]);

  useEffect(() => {
    if (currentActionNode && color) {
      var rgb: [number, number, number] = [
        color.rgb["r"],
        color.rgb["g"],
        color.rgb["b"],
      ];

      var actionFrame = getActionFrame(currentActionNode.getName());

      changeColorNode(
        rgb,
        actionFrame,
        currentActionNode,
        engine,
        model,
        () => {},
        updateJsonState
      );
    }
  }, [color]);

  const isBackgroundDark = () => {
    if (!color) {
      return false;
    }

    return (
      rgbToLuminance(color.rgb["r"], color.rgb["g"], color.rgb["b"]) <= 0.5
    );
  };

  const reRender = () => {
    document.getElementById("node-editor-modal")!.focus();
  };

  const horizontalScrolling = (e: any) => {
    e.preventDefault();
    // var containerScrollPosition = e.target.scrollLeft;
    e.target.scrollBy({
      top: 0,
      left: e.deltaY,
      behaviour: "smooth",
    });
  };

  const openInputCreation = () => {
    if (!outputName) {
      setInputName(true);
      setAllowCreation(false);
    }
  };

  const openOutputCreation = () => {
    if (!inputName) {
      setOutputName(true);
      setAllowCreation(false);
    }
  };

  const isInputNameValid = (name: string) => {
    var inputPorts = Object.entries(currentActionNode.getPorts()).filter(
      (item) => item[1] instanceof InputPortModel
    );
    var merged = [].concat.apply(
      inputPorts.map((x) => x[0]),
      []
    );
    return (
      name !== "" && !name.includes(" ") && !merged.includes(name as never)
    );
  };

  const isOutputNameValid = (name: string) => {
    var outputPorts = Object.entries(currentActionNode.getPorts()).filter(
      (item) => item[1] instanceof OutputPortModel
    );
    var merged = [].concat.apply(
      outputPorts.map((x) => x[0]),
      []
    );
    return (
      name !== "" && !name.includes(" ") && !merged.includes(name as never)
    );
  };

  const addInput = () => {
    //TODO: Maybe display some error message when the name is invalid
    if (isInputNameValid(formState["newInputName"])) {
      var actionFrame = getActionFrame(currentActionNode.getName());

      addPort(
        formState["newInputName"],
        actionFrame,
        currentActionNode,
        ActionNodePortType.Input,
        engine,
        model,
        () => {},
        updateJsonState
      );
    }
    setInputName(false);
    reRender();
  };

  const addOutput = () => {
    //TODO: Maybe display some error message when the name is invalid
    if (isOutputNameValid(formState["newOutputName"])) {
      var actionFrame = getActionFrame(currentActionNode.getName());

      addPort(
        formState["newOutputName"],
        actionFrame,
        currentActionNode,
        ActionNodePortType.Output,
        engine,
        model,
        () => {},
        updateJsonState
      );
    }
    setOutputName(false);
    reRender();
  };

  const removeInput = (port: InputPortModel) => {
    var actionFrame = getActionFrame(currentActionNode.getName());

    removePort(
      port,
      actionFrame,
      currentActionNode,
      engine,
      model,
      () => {},
      updateJsonState
    );

    setUpdate(true);
  };

  const removeOutput = (port: OutputPortModel) => {
    var actionFrame = getActionFrame(currentActionNode.getName());

    removePort(
      port,
      actionFrame,
      currentActionNode,
      engine,
      model,
      () => {},
      updateJsonState
    );

    setUpdate(true);
  };

  const cancelCreation = () => {
    setInputName(false);
    setOutputName(false);
    reRender();
  };

  return (
    <Modal
      id="node-editor-modal"
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
          Edit action value
        </label>
        <CloseIcon
          className="bt-modal-titlebar-close bt-icon"
          onClick={() => {
            onClose();
          }}
          fill={"var(--icon)"}
        />
      </div>
      <div className="bt-node-editor-row">
        {currentActionNode && (
          <div
            className="bt-node-editor"
            style={{ backgroundColor: currentActionNode.getColor() }}
          >
            <label
              className="bt-node-editor-name"
              style={{
                color: isBackgroundDark()
                  ? "var(--bt-light-text)"
                  : "var(--bt-dark-text)",
              }}
            >
              {currentActionNode.getName()}
            </label>
            <div className="bt-node-editor-io">
              <div className="bt-node-editor-inputs">
                {Object.entries(currentActionNode.getPorts()).map(
                  (port, index) => {
                    if (port[1] instanceof InputPortModel) {
                      return (
                        <div
                          key={index}
                          className="bt-node-editor-input bt-node-editor-io-entry"
                        >
                          <label
                            id={port[0]}
                            className="bt-node-editor-io-name"
                            onWheel={horizontalScrolling}
                            style={{
                              color: isBackgroundDark()
                                ? "var(--bt-light-text)"
                                : "var(--bt-dark-text)",
                            }}
                          >
                            {port[0]}
                          </label>
                          <button
                            className={
                              "bt-node-editor-io-delete bt-node-editor-hidden"
                            }
                            style={{
                              color: isBackgroundDark()
                                ? "var(--bt-light-text)"
                                : "var(--bt-dark-text)",
                            }}
                            title="Delete"
                            onClick={() => {
                              removeInput(port[1] as InputPortModel);
                            }}
                          >
                            <DeleteIcon
                              className="bt-icon"
                              fill={"var(--icon)"}
                              style={{
                                filter: isBackgroundDark()
                                  ? "invert(0)"
                                  : "invert(1)",
                              }}
                            />
                          </button>
                        </div>
                      );
                    }
                    return <></>;
                  }
                )}
                {inputName ? (
                  <div className="bt-node-editor-io-name-entry-container">
                    <input
                      ref={focusInputRef}
                      type="text"
                      id="node-editor-io-name-entry"
                      name="newInputName"
                      className="bt-node-editor-io-name-entry"
                      autoComplete="off"
                      onChange={handleInputChange}
                      required
                      style={{
                        color: isBackgroundDark()
                          ? "var(--bt-light-text)"
                          : "var(--bt-dark-text)",
                      }}
                    />
                    <button
                      className={"bt-node-editor-io-delete"}
                      style={{
                        color: isBackgroundDark()
                          ? "var(--bt-light-text)"
                          : "var(--bt-dark-text)",
                      }}
                      title="Cancel"
                      onClick={() => cancelCreation()}
                    >
                      <CancelIcon
                        className="bt-icon"
                        fill={"var(--icon)"}
                        style={{
                          filter: isBackgroundDark()
                            ? "invert(0)"
                            : "invert(1)",
                        }}
                      />
                    </button>
                    {allowCreation && (
                      <button
                        className={"bt-node-editor-io-accept"}
                        style={{
                          color: isBackgroundDark()
                            ? "var(--bt-light-text)"
                            : "var(--bt-dark-text)",
                        }}
                        title="Create"
                        onClick={() => addInput()}
                      >
                        <AcceptIcon
                          className="bt-icon"
                          fill={"var(--icon)"}
                          style={{
                            filter: isBackgroundDark()
                              ? "invert(0)"
                              : "invert(1)",
                          }}
                        />
                      </button>
                    )}
                  </div>
                ) : (
                  <button
                    className="bt-node-editor-button"
                    style={{
                      color: isBackgroundDark()
                        ? "var(--bt-light-text)"
                        : "var(--bt-dark-text)",
                    }}
                    onClick={() => {
                      openInputCreation();
                    }}
                    title="Add input"
                  >
                    <AddIcon
                      className="bt-icon bt-action-icon"
                      fill={"var(--icon)"}
                      style={{
                        filter: isBackgroundDark() ? "invert(0)" : "invert(1)",
                      }}
                    />
                  </button>
                )}
              </div>
              <div className="bt-node-editor-outputs">
                {Object.entries(currentActionNode.getPorts()).map(
                  (port, index) => {
                    if (port[1] instanceof OutputPortModel) {
                      return (
                        <div
                          key={index}
                          className="bt-node-editor-output bt-node-editor-io-entry"
                        >
                          <button
                            className={
                              "bt-node-editor-io-delete bt-node-editor-hidden"
                            }
                            style={{
                              color: isBackgroundDark()
                                ? "var(--bt-light-text)"
                                : "var(--bt-dark-text)",
                            }}
                            title="Delete"
                            onClick={() => {
                              removeOutput(port[1] as OutputPortModel);
                            }}
                          >
                            <DeleteIcon
                              className="bt-icon"
                              fill={"var(--icon)"}
                              style={{
                                filter: isBackgroundDark()
                                  ? "invert(0)"
                                  : "invert(1)",
                              }}
                            />
                          </button>
                          <label
                            className="bt-node-editor-io-name"
                            onWheel={horizontalScrolling}
                            style={{
                              color: isBackgroundDark()
                                ? "var(--bt-light-text)"
                                : "var(--bt-dark-text)",
                            }}
                          >
                            {port[0]}
                          </label>
                        </div>
                      );
                    }
                    return <></>;
                  }
                )}
                {outputName ? (
                  <div className="bt-node-editor-io-name-entry-container">
                    <input
                      ref={focusInputRef}
                      type="text"
                      id="node-editor-io-name-entry"
                      name="newOutputName"
                      className="bt-node-editor-io-name-entry"
                      autoComplete="off"
                      onChange={handleInputChange}
                      required
                      style={{
                        color: isBackgroundDark()
                          ? "var(--bt-light-text)"
                          : "var(--bt-dark-text)",
                      }}
                    />
                    <button
                      className={"bt-node-editor-io-delete"}
                      style={{
                        color: isBackgroundDark()
                          ? "var(--bt-light-text)"
                          : "var(--bt-dark-text)",
                      }}
                      title="Cancel"
                      onClick={() => cancelCreation()}
                    >
                      <CancelIcon
                        className="bt-icon"
                        fill={"var(--icon)"}
                        style={{
                          filter: isBackgroundDark()
                            ? "invert(0)"
                            : "invert(1)",
                        }}
                      />
                    </button>
                    {allowCreation && (
                      <button
                        className={"bt-node-editor-io-accept"}
                        style={{
                          color: isBackgroundDark()
                            ? "var(--bt-light-text)"
                            : "var(--bt-dark-text)",
                        }}
                        title="Create"
                        onClick={() => addOutput()}
                      >
                        <AcceptIcon
                          className="bt-icon"
                          fill={"var(--icon)"}
                          style={{
                            filter: isBackgroundDark()
                              ? "invert(0)"
                              : "invert(1)",
                          }}
                        />
                      </button>
                    )}
                  </div>
                ) : (
                  <button
                    className="bt-node-editor-button"
                    style={{
                      color: isBackgroundDark()
                        ? "var(--bt-light-text)"
                        : "var(--bt-dark-text)",
                    }}
                    onClick={() => {
                      openOutputCreation();
                    }}
                    title="Add output"
                  >
                    <AddIcon
                      className="bt-icon bt-action-icon"
                      fill={"var(--icon)"}
                      style={{
                        filter: isBackgroundDark() ? "invert(0)" : "invert(1)",
                      }}
                    />
                  </button>
                )}
              </div>
            </div>
          </div>
        )}
      </div>
      {color && (
        <div className="bt-node-editor-row">
          <label className="bt-node-editor-title" htmlFor="favcolor">
            Color:
          </label>
          <Saturation height={50} color={color} onChange={setColor} />
          <Hue color={color} onChange={setColor} />
        </div>
      )}
    </Modal>
  );
};

export default EditActionModal;
