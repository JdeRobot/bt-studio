import React, { useState, useEffect, useRef } from 'react';
import { Saturation, Hue, useColor } from "react-color-palette";
import "react-color-palette/css";
import './SettingsModal.css';
import Modal from '../Modal/Modal';
import close_modal_img from '../Modal/img/close.svg'
import Dropdown from './options/Dropdown';
import Checkbox from './options/Checkbox';

const SettingsModal = ({ onSubmit, isOpen, onClose, settings}) => {
  const [color, setColor] = useColor("rgb(128 0 128)");
  const [open, setOpen] = useState(false);


  useEffect(() => {
  }, [isOpen]);

  // useEffect(() => {
  //   console.log("rgb("+Math.round(color.rgb.r)+","+Math.round(color.rgb.g)+","+Math.round(color.rgb.b)+")")
  //   document.documentElement.style.setProperty("--header", "rgb("+Math.round(color.rgb.r)+","+Math.round(color.rgb.g)+","+Math.round(color.rgb.b)+")");
  // }, [color]);

  const handleCancel = () => {
    onClose()
  };

  // Section: Collapsable
  // Subsection: plain text
  // Setting: all of them

  return (
    <Modal id="settings-modal" hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={onSubmit} onReset={handleCancel}>
        <div className="modal-titlebar">
          <label className='modal-titlebar-title' htmlFor="actionName" style={{ textAlign: "center" }}>Settings</label>
          <img className="modal-titlebar-close" onClick={() => { handleCancel(); } } src={close_modal_img}></img>
        </div>
        <div className="form-row">
            <ul className='settings-entry-list'>
              {/* <div className='setting-section'>
                <label className='setting-section-title'>General</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Accent Colors</label>
                    <div className='setting-setting'>
                      <label className='setting-setting-title'>Turn on project accent color</label>
                      Checkbox here
                      <input type="checkbox" className='setting-setting-checkbox' checked={settings.theme} onChange={() => settings.setTheme(!settings.theme)}/>
                    </div>
                    Only show next if above is on
                    { false &&
                    <div className='setting-setting' id="setting-project-accent-color">
                      <label className='setting-setting-title'>Project accent color</label>
                      <Saturation height={100} color={color} onChange={setColor} />
                      <Hue width={300} color={color} onChange={setColor} />
                    </div>
                    }
                </div>
              </div> */}
              <div className='setting-section'>
                <label className='setting-section-title'>Style</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Color theme</label>
                  <div className='setting-setting'>
                    <label className='setting-setting-title'>Dark mode enabled</label>
                    <Dropdown
                      value={settings.theme}
                      setValue={settings.setTheme}
                      possibleValues={["dark", "light"]}
                    />
                  </div>
                </div>
              </div>
              <div className='setting-section'>
                <label className='setting-section-title'>Editor</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Accent Color</label>
                  <div className='setting-setting'>
                    <label className='setting-setting-title'>Show actions accent color</label>
                    <Checkbox
                      value={settings.editor.accentColors}
                      setValue={settings.editor.setAccentColors}
                    />
                  </div>
                </div>
              </div>
              <div className='setting-section'>
                <label className='setting-section-title'>Behaviour Tree</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Execution settings</label>
                  <div className='setting-setting'>
                    <label className='setting-setting-title'>Order of execution of the behavior tree</label>
                    {/* Add explanation here */}
                    <Dropdown
                      value={settings.btOrder}
                      setValue={settings.setBtOrder}
                      possibleValues={["bottom-to-top", "top-to-bottom"]}
                    />
                  </div>
                </div>
              </div>
            </ul>
          </div>
      </form>
    </Modal>
  );
};

export default SettingsModal;