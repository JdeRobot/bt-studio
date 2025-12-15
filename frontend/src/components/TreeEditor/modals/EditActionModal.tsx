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

import {
  contrastSelector,
  Modal,
  ModalRow,
  ModalTitlebar,
} from "jderobot-ide-interface";
import { BasicNodeModel } from "../nodes/basic_node/BasicNodeModel";
import { OutputPortModel } from "../nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "../nodes/basic_node/ports/input_port/InputPortModel";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledBTModelAdd,
  StyledBTModelContainer,
  StyledBTModelInput,
  StyledBTModelInputContainer,
  StyledBTModelIO,
  StyledBTModelIODelete,
  StyledBTModelIOEntry,
  StyledBTModelName,
} from "BtStyles/TreeEditor/BTModals.styles";
import { rgbToHex } from "@mui/material";

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
  const theme = useBtTheme();

  const focusInputRef = useRef(null);
  const [color, setColor] = useState<IColor | undefined>(undefined);
  const [inputName, setInputName] = React.useState(false);
  const [outputName, setOutputName] = React.useState(false);
  const [allowCreation, setAllowCreation] = React.useState(false);
  const [formState, setFormState] = useState(initialEditActionModalData);
  const [update, setUpdate] = React.useState(false);

  const handleInputChange = (event: any) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    setAllowCreation(
      (name === "newInputName" && isInputNameValid(value)) ||
        (name === "newOutputName" && isOutputNameValid(value)),
    );
  };

  useEffect(() => {
    setInputName(false);
    setOutputName(false);
    setFormState(initialEditActionModalData);
    document.getElementById("node-editor-modal")!.focus();
    if (currentActionNode) {
      const rgb: IColor["rgb"] = ColorService.toRgb(
        currentActionNode.getColor(),
      );

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
      const rgb: [number, number, number] = [
        color.rgb["r"],
        color.rgb["g"],
        color.rgb["b"],
      ];

      const actionFrame = getActionFrame(currentActionNode.getName());

      changeColorNode(
        rgb,
        actionFrame,
        currentActionNode,
        engine,
        model,
        () => {},
        setFileContent,
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
    const inputPorts = Object.entries(currentActionNode.getPorts()).filter(
      (item) => item[1] instanceof InputPortModel,
    );
    const merged = [].concat.apply(
      inputPorts.map((x) => x[0]),
      [],
    );
    return (
      name !== "" && !name.includes(" ") && !merged.includes(name as never)
    );
  };

  const isOutputNameValid = (name: string) => {
    const outputPorts = Object.entries(currentActionNode.getPorts()).filter(
      (item) => item[1] instanceof OutputPortModel,
    );
    const merged = [].concat.apply(
      outputPorts.map((x) => x[0]),
      [],
    );
    return (
      name !== "" && !name.includes(" ") && !merged.includes(name as never)
    );
  };

  const addInput = () => {
    //TODO: Maybe display some error message when the name is invalid
    if (isInputNameValid(formState["newInputName"])) {
      const actionFrame = getActionFrame(currentActionNode.getName());

      addPort(
        formState["newInputName"],
        actionFrame,
        currentActionNode,
        ActionNodePortType.Input,
        engine,
        model,
        () => {},
        setFileContent,
      );
    }
    setInputName(false);
    reRender();
  };

  const addOutput = () => {
    //TODO: Maybe display some error message when the name is invalid
    if (isOutputNameValid(formState["newOutputName"])) {
      const actionFrame = getActionFrame(currentActionNode.getName());

      addPort(
        formState["newOutputName"],
        actionFrame,
        currentActionNode,
        ActionNodePortType.Output,
        engine,
        model,
        () => {},
        setFileContent,
      );
    }
    setOutputName(false);
    reRender();
  };

  const removeInput = (port: InputPortModel) => {
    const actionFrame = getActionFrame(currentActionNode.getName());

    removePort(
      port,
      actionFrame,
      currentActionNode,
      engine,
      model,
      () => {},
      setFileContent,
    );

    setUpdate(true);
  };

  const removeOutput = (port: OutputPortModel) => {
    const actionFrame = getActionFrame(currentActionNode.getName());

    removePort(
      port,
      actionFrame,
      currentActionNode,
      engine,
      model,
      () => {},
      setFileContent,
    );

    setUpdate(true);
  };

  const cancelCreation = () => {
    setInputName(false);
    setOutputName(false);
    reRender();
  };

  const btTheme = theme.btEditor;
  const bg = currentActionNode.getColor();

  const cancelColor = contrastSelector(
    btTheme.lightText,
    btTheme.darkText,
    btTheme.failure,
  );

  const addColor = contrastSelector(
    btTheme.lightText,
    btTheme.darkText,
    btTheme.success,
  );

  const text = contrastSelector(
    btTheme.lightText,
    btTheme.darkText,
    rgbToHex(currentActionNode.getColor()),
  );

  return (
    <Modal id="node-editor-modal" isOpen={isOpen} onClose={onClose}>
      <ModalTitlebar
        title="Edit action value"
        htmlFor="actionName"
        hasClose
        handleClose={() => {
          onClose();
        }}
      />
      <ModalRow type="all">
        <StyledBTModelContainer
          bg={currentActionNode.getColor()}
          borderColor={btTheme.border}
          roundness={btTheme.roundness}
          lightText={btTheme.lightText}
          darkText={btTheme.darkText}
        >
          <StyledBTModelName>{currentActionNode.getName()}</StyledBTModelName>
          <StyledBTModelIO>
            <div>
              {Object.entries(currentActionNode.getPorts()).map(
                (port, index) => {
                  if (port[1] instanceof InputPortModel) {
                    return (
                      <StyledBTModelIOEntry
                        bg={bg}
                        roundness={btTheme.roundness}
                        type="input"
                        key={index}
                      >
                        <label id={port[0]} onWheel={horizontalScrolling}>
                          {port[0]}
                        </label>
                        <StyledBTModelIODelete
                          roundness={btTheme.roundness}
                          bg={btTheme.failure}
                          title="Delete"
                          onClick={() => {
                            removeInput(port[1] as InputPortModel);
                          }}
                        >
                          <DeleteIcon fill={cancelColor} />
                        </StyledBTModelIODelete>
                      </StyledBTModelIOEntry>
                    );
                  }
                  return <></>;
                },
              )}
              {inputName ? (
                <StyledBTModelInputContainer roundness={btTheme.roundness}>
                  <StyledBTModelInput
                    ref={focusInputRef}
                    type="text"
                    id="node-editor-io-name-entry"
                    name="newInputName"
                    autoComplete="off"
                    onChange={handleInputChange}
                    required
                    roundness={btTheme.roundness}
                    bg={theme.palette.bg}
                    color={theme.palette.text}
                  />
                  <StyledBTModelIODelete
                    roundness={btTheme.roundness}
                    bg={btTheme.failure}
                    visible
                    title="Cancel"
                    onClick={() => cancelCreation()}
                  >
                    <CancelIcon fill={cancelColor} />
                  </StyledBTModelIODelete>
                  {allowCreation && (
                    <StyledBTModelIODelete
                      roundness={btTheme.roundness}
                      bg={btTheme.success}
                      visible
                      title="Create"
                      onClick={() => addInput()}
                    >
                      <AcceptIcon fill={addColor} />
                    </StyledBTModelIODelete>
                  )}
                </StyledBTModelInputContainer>
              ) : (
                <AddButton onClick={openInputCreation} color={text} bg={bg} />
              )}
            </div>
            <div>
              {Object.entries(currentActionNode.getPorts()).map(
                (port, index) => {
                  if (port[1] instanceof OutputPortModel) {
                    return (
                      <StyledBTModelIOEntry
                        bg={bg}
                        roundness={theme.btEditor.roundness}
                        type="output"
                        key={index}
                      >
                        <StyledBTModelIODelete
                          roundness={btTheme.roundness}
                          bg={btTheme.failure}
                          title="Delete"
                          onClick={() => {
                            removeOutput(port[1] as OutputPortModel);
                          }}
                        >
                          <DeleteIcon fill={cancelColor} />
                        </StyledBTModelIODelete>
                        <label id={port[0]} onWheel={horizontalScrolling}>
                          {port[0]}
                        </label>
                      </StyledBTModelIOEntry>
                    );
                  }
                  return <></>;
                },
              )}
              {outputName ? (
                <StyledBTModelInputContainer roundness={btTheme.roundness}>
                  <StyledBTModelInput
                    ref={focusInputRef}
                    type="text"
                    id="node-editor-io-name-entry"
                    name="newOutputName"
                    autoComplete="off"
                    onChange={handleInputChange}
                    required
                    roundness={btTheme.roundness}
                    bg={theme.palette.bg}
                    color={theme.palette.text}
                  />
                  <StyledBTModelIODelete
                    roundness={btTheme.roundness}
                    bg={btTheme.failure}
                    visible
                    title="Cancel"
                    onClick={() => cancelCreation()}
                  >
                    <CancelIcon fill={cancelColor} />
                  </StyledBTModelIODelete>
                  {allowCreation && (
                    <StyledBTModelIODelete
                      roundness={btTheme.roundness}
                      bg={btTheme.success}
                      visible
                      title="Create"
                      onClick={() => addOutput()}
                    >
                      <AcceptIcon fill={addColor} />
                    </StyledBTModelIODelete>
                  )}
                </StyledBTModelInputContainer>
              ) : (
                <AddButton onClick={openOutputCreation} color={text} bg={bg} />
              )}
            </div>
          </StyledBTModelIO>
        </StyledBTModelContainer>
      </ModalRow>
      {color && (
        <ModalRow type="all">
          <StyledBTModelName>Color:</StyledBTModelName>
          <Saturation height={50} color={color} onChange={setColor} />
          <Hue color={color} onChange={setColor} />
        </ModalRow>
      )}
    </Modal>
  );
};

const AddButton = ({
  onClick,
  color,
  bg,
}: {
  onClick: () => void;
  color?: string;
  bg: string;
}) => {
  const theme = useBtTheme();

  return (
    <StyledBTModelAdd
      roundness={theme.btEditor.roundness}
      bg={bg}
      onClick={onClick}
      title="Add input"
    >
      <AddIcon fill={color ?? theme.btEditor.lightText} />
    </StyledBTModelAdd>
  );
};

export default EditActionModal;
