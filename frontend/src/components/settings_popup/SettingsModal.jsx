import React, { useState, useEffect, useRef } from 'react';
import { Saturation, Hue, useColor } from "react-color-palette";
import "react-color-palette/css";
import './SettingsModal.css';
import Modal from '../Modal/Modal';
import close_modal_img from '../Modal/img/close.svg'

import Section from './sections/Section';
import SubSection from './sections/SubSection';
import Setting from './sections/Setting';

import Dropdown from './options/Dropdown';
import Checkbox from './options/Checkbox';

const SettingsModal = ({ onSubmit, isOpen, onClose, settings}) => {
  const [color, setColor] = useColor("rgb(128 0 128)");
  const [open, setOpen] = useState(false);


  useEffect(() => {
    // Load settings
  }, [isOpen]);

  // useEffect(() => {
  //   console.log("rgb("+Math.round(color.rgb.r)+","+Math.round(color.rgb.g)+","+Math.round(color.rgb.b)+")")
  //   document.documentElement.style.setProperty("--header", "rgb("+Math.round(color.rgb.r)+","+Math.round(color.rgb.g)+","+Math.round(color.rgb.b)+")");
  // }, [color]);

  const handleCancel = () => {
    // Save settings
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
        <div className="form-row" style={{height:"95%"}}>
            <ul className='settings-entry-list'>
              {/* <Section title="General">
                <SubSection title="Accent Colors">
                  <Setting title ="Turn on project accent color">
                  </Setting>
                  <Setting title ="Project accent color">
                    <Saturation height={100} color={color} onChange={setColor} />
                    <Hue width={300} color={color} onChange={setColor} />
                  </Setting>
                </SubSection>
              </Section> */}
              <Section title="Appearance">
                <SubSection title="Color theme">
                  <Setting title ="Set color theme">
                    <Dropdown
                      value={settings.theme}
                      setValue={settings.setTheme}
                      possibleValues={["dark", "light"]}
                    />
                  </Setting>
                </SubSection>
                <SubSection title="Editor">
                  <Setting title ="Show actions accent color">
                    <Checkbox
                      value={settings.editor.accentColors}
                      setValue={settings.editor.setAccentColors}
                    />
                  </Setting>
                </SubSection>
              </Section>
              <Section title="Behaviour Tree">
                <SubSection title="Execution settings">
                  <Setting title ="Order of execution of the behavior tree">
                    {/* Add explanation here */}
                    <Dropdown
                      value={settings.btOrder}
                      setValue={settings.setBtOrder}
                      possibleValues={["bottom-to-top", "top-to-bottom"]}
                    />
                  </Setting>
                </SubSection>
              </Section>
            </ul>
          </div>
      </form>
    </Modal>
  );
};

export default SettingsModal;