import React, { useState, useEffect, useRef } from 'react';

import { Saturation, Hue, useColor } from "react-color-palette";
import "react-color-palette/css";

import './EditActionModal.css';
import Modal from '../../Modal/Modal';
import { OutputPortModel } from '../nodes/basic_node/ports/output_port/OutputPortModel';
import { InputPortModel } from '../nodes/basic_node/ports/input_port/InputPortModel';

import add_icon from '../img/add.svg';
import delete_icon from '../img/delete.svg';
import cancel_icon from '../img/cancel.svg';
import accept_icon from '../img/accept.svg';
import close_modal_img from '../../Modal/img/close.svg'

const initialEditActionModalData = {
  newInputName: '',
  newOutputName: '',
};

function rgb2hsv (r, g, b) {
    let rabs, gabs, babs, rr, gg, bb, h, s, v, diff, diffc, percentRoundFn;
    rabs = r / 255;
    gabs = g / 255;
    babs = b / 255;
    v = Math.max(rabs, gabs, babs);
    diff = v - Math.min(rabs, gabs, babs);
    diffc = c => (v - c) / 6 / diff + 1 / 2;
    percentRoundFn = num => Math.round(num * 100) / 100;
    if (diff == 0) {
        h = s = 0;
    } else {
        s = diff / v;
        rr = diffc(rabs);
        gg = diffc(gabs);
        bb = diffc(babs);

        if (rabs === v) {
            h = bb - gg;
        } else if (gabs === v) {
            h = (1 / 3) + rr - bb;
        } else if (babs === v) {
            h = (2 / 3) + gg - rr;
        }
        if (h < 0) {
            h += 1;
        }else if (h > 1) {
            h -= 1;
        }
    }
    return {
        h: Math.round(h * 360),
        s: percentRoundFn(s * 100),
        v: percentRoundFn(v * 100)
    };
}

const EditActionModal = ({ isOpen, onClose, currentActionNode, setColorActionNode, addInputPort, addOutputPort, deleteInputPort, deleteOutputPort}) => {
  const focusInputRef = useRef(null);
  const [color, setColor] = useColor("rgb(128 0 128)");
  const [inputName, setInputName] = React.useState(false);
  const [outputName, setOutputName] = React.useState(false);
  const [allowCreation, setAllowCreation] = React.useState(false);
  const [formState, setFormState] = useState(initialEditActionModalData);
  const [, updateState] = React.useState();
  const forceUpdate = React.useCallback(() => updateState({}), []);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    setAllowCreation((name === "newInputName" && isInputNameValid(value)) || (name === "newOutputName" && isOutputNameValid(value)));
  };

  useEffect(() => {
    setInputName(false)
    setOutputName(false)
    setFormState(initialEditActionModalData);
    document.getElementById('node-editor-modal').focus();
    if (currentActionNode) {
      var colorRGB = currentActionNode.getColor().split('(')[1].split(')')[0].split(',')
      color.rgb['r'] = colorRGB[0]
      color.rgb['g'] = colorRGB[1]
      color.rgb['b'] = colorRGB[2]
      var hsvColor = rgb2hsv(colorRGB[0], colorRGB[1], colorRGB[2]);
      color.hsv['h'] = hsvColor['h']
      color.hsv['s'] = hsvColor['s']
      color.hsv['v'] = hsvColor['v']
      setColor(color)
      reRender();
    }
  }, [isOpen]);

  useEffect(() => {
    if (currentActionNode) {
      setColorActionNode(color.rgb['r'], color.rgb['g'], color.rgb['b']);
    }
  }, [color]);

  const isBackgroundDark = () => {
    return color.hsv['v'] < 60 || color.hsv['s'] > 55
  }

  const reRender = () => {
    forceUpdate();
    document.getElementById('node-editor-modal').focus();
  }

  const horizontalScrolling = (e) => {
    e.preventDefault()
    var containerScrollPosition = e.target.scrollLeft
    e.target.scrollBy({
        top: 0,
        left: e.deltaY,
        behaviour: 'smooth'
    })
  }

  const openInputCreation = () => {
    if (!outputName) {
      setInputName(true)
      setAllowCreation(false)
    }
  }

  const openOutputCreation = () => {
    if (!inputName) {
      setOutputName(true)
      setAllowCreation(false)
    }
  }

  const isInputNameValid = (name) => {
    var inputPorts = Object.entries(currentActionNode.getPorts()).filter(item => item[1] instanceof InputPortModel);
    var merged = [].concat.apply([], inputPorts);
    return (name !== '' && !name.includes(' ') && !merged.includes(name))
  }

  const isOutputNameValid = (name) => {
    var outputPorts = Object.entries(currentActionNode.getPorts()).filter(item => item[1] instanceof OutputPortModel);
    var merged = [].concat.apply([], outputPorts);
    return (name !== '' && !name.includes(' ') && !merged.includes(name))
  }

  const addInput = () => {
    //TODO: Maybe display some error message when the name is invalid
    if (isInputNameValid(formState['newInputName'])) {
      addInputPort(formState['newInputName']);
    }
    setInputName(false);
    reRender();
  }

  const addOutput = () => {
    //TODO: Maybe display some error message when the name is invalid
    if (isOutputNameValid(formState['newOutputName'])) {
      addOutputPort(formState['newOutputName']);
    }
    setOutputName(false);
    reRender();
  }

  const cancelCreation = () => {
    setInputName(false)
    setOutputName(false)
    reRender();
  }

  return (
    <Modal id="node-editor-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose} >
      <div className="modal-titlebar">
        <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Edit action value</label>
        <img className="modal-titlebar-close" onClick={() => { onClose(); } } src={close_modal_img}></img>
      </div>
      <div className="node-editor-row">
        {currentActionNode &&
          <div className="node-editor" style={{backgroundColor: currentActionNode.getColor()}}>
            <label className="node-editor-name" style={{color: isBackgroundDark() ? 'white' : 'black'}}>{currentActionNode.getName()}</label>
            <div className="node-editor-io">
              <div className="node-editor-inputs">
                {Object.entries(currentActionNode.getPorts()).map((port, index) => {
                  if (port[1] instanceof InputPortModel) {
                    return (
                      <div key={index} className="node-editor-input node-editor-io-entry">
                        <label
                          id={port[0]}
                          className="node-editor-io-name"
                          onWheel={horizontalScrolling}
                          style={{color: isBackgroundDark() ? 'white' : 'black'}}>{port[0]}</label>
                        <button
                          className={"node-editor-io-delete node-editor-hidden"}
                          style={{color: isBackgroundDark() ? 'white' : 'black'}}
                          title='Delete'
                          onClick={() => {deleteInputPort(port[1], port[0]); reRender()}}>
                          <img className="icon" src={delete_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>
                        </button>
                      </div>
                    );
                  }
                })}
                {inputName ? (
                  <div className="node-editor-io-name-entry-container">
                      <input
                        ref={focusInputRef}
                        type="text"
                        id="node-editor-io-name-entry"
                        name="newInputName"
                        className='node-editor-io-name-entry'
                        autoComplete='off'
                        onChange={handleInputChange}
                        required
                        style={{color: isBackgroundDark() ? 'white' : 'black'}}
                      />
                      <button
                        className={"node-editor-io-delete"}
                        style={{color: isBackgroundDark() ? 'white' : 'black'}}
                        title='Cancel'
                        onClick={() => cancelCreation()}>
                        <img className="icon" src={cancel_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>  
                      </button>
                      { allowCreation &&
                        <button
                          className={"node-editor-io-accept"}
                          style={{color: isBackgroundDark() ? 'white' : 'black'}}
                          title='Create'
                          onClick={() => addInput()}>
                          <img className="icon" src={accept_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>
                        </button>
                      }
                  </div>
                    )
                    : (
                  <button
                    className="node-editor-button"
                    style={{color: isBackgroundDark() ? 'white' : 'black'}}
                    onClick={() => {openInputCreation()}}
                    title='Add input'>
                    <img className="icon action-icon" src={add_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>
                  </button>
                    )
                  }
              </div>
              <div className="node-editor-outputs">
                {Object.entries(currentActionNode.getPorts()).map((port, index) => {
                  if (port[1] instanceof OutputPortModel) {
                    return (
                      <div key={index} className="node-editor-output node-editor-io-entry" >
                        <button
                          className={"node-editor-io-delete node-editor-hidden"}
                          style={{color: isBackgroundDark() ? 'white' : 'black'}}
                          title='Delete'
                          onClick={() => {deleteOutputPort(port[1], port[0]); reRender()}}>
                          <img className="icon" src={delete_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>
                        </button>
                        <label
                          className="node-editor-io-name"
                          onWheel={horizontalScrolling}
                          style={{color: isBackgroundDark() ? 'white' : 'black'}}>{port[0]}</label>
                      </div>
                    );
                  }
                })}
                {outputName ? (
                  <div className="node-editor-io-name-entry-container">
                      <input
                        ref={focusInputRef}
                        type="text"
                        id="node-editor-io-name-entry"
                        name="newOutputName"
                        className='node-editor-io-name-entry'
                        autoComplete='off'
                        onChange={handleInputChange}
                        required
                        style={{color: isBackgroundDark() ? 'white' : 'black'}}
                      />
                      <button
                        className={"node-editor-io-delete"}
                        style={{color: isBackgroundDark() ? 'white' : 'black'}}
                        title='Cancel'
                        onClick={() => cancelCreation()}>
                        <img className="icon" src={cancel_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>  
                      </button>
                      { allowCreation &&
                        <button
                          className={"node-editor-io-accept"}
                          style={{color: isBackgroundDark() ? 'white' : 'black'}}
                          title='Create'
                          onClick={() => addOutput()}>
                          <img className="icon" src={accept_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>
                        </button>
                      }
                  </div>
                    )
                    : (
                  <button
                    className="node-editor-button"
                    style={{color: isBackgroundDark() ? 'white' : 'black'}}
                    onClick={() => {openOutputCreation()}}
                    title='Add output'>
                    <img className="icon action-icon" src={add_icon} style={{filter: isBackgroundDark() ? 'invert(0)' : 'invert(1)'}}></img>
                  </button>
                    )
                  }
              </div>
            </div>
          </div>
        }
      </div>
      <div className="node-editor-row">
        <label className="node-editor-title" for="favcolor">Color:</label>
        <Saturation height={50} width={300} color={color} onChange={setColor} />
        <Hue color={color} onChange={setColor} />
      </div>
    </Modal>
  );
};

export default EditActionModal;