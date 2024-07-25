import React, { useState, useEffect, useRef } from 'react';
import './SettingsModal.css';
import Modal from '../Modal/Modal';
import close_modal_img from '../Modal/img/close.svg'

const SettingsModal = ({ onSubmit, isOpen, onClose, isDarkMode, enableDarkMode}) => {

  useEffect(() => {
  }, [isOpen]);

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
              <div className='setting-section'>
                <label className='setting-section-title'>General</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Accent Colors</label>
                    <div className='setting-setting'>
                      <label className='setting-setting-title'>Turn on project accent color</label>
                      {/* Checkbox here */}
                      <input type="checkbox" className='setting-setting-checkbox' checked={isDarkMode} onChange={() => enableDarkMode(!isDarkMode)}/>
                    </div>
                    {/* Only show next if above is on */}
                    <div className='setting-setting'>
                      <label className='setting-setting-title'>Project accent color</label>
                      {/* Color selection here */}
                    </div>
                </div>
              </div>
              <div className='setting-section'>
                <label className='setting-section-title'>Style</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Color theme</label>
                  <div className='setting-setting'>
                    <label className='setting-setting-title'>Dark mode enabled</label>
                    {/* Options here */}
                    <input type="checkbox" className='setting-setting-checkbox' checked={isDarkMode} onChange={() => enableDarkMode(!isDarkMode)}/>
                  </div>
                </div>
              </div>
              <div className='setting-section'>
                <label className='setting-section-title'>Behaviour Tree</label>
                <div className='setting-subsection'>
                  <label className='setting-subsection-title'>Execution settings</label>
                  <div className='setting-setting'>
                    <label className='setting-setting-title'>Order of execution of the behavior tree</label>
                    {/* Options here */}
                    {/* Add explanation here */}
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