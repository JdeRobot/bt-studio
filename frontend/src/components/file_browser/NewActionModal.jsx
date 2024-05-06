import React, { useState, useEffect, useRef } from 'react';
import './NewActionModal.css';
import Modal from '../Modal/Modal';
import empty_template from './img/empty_template.svg'

const initialNewActionModalData = {
  actionName: '',
  templateType: 'empty',
  allowCreation: false,
};

function CreateButton({ hasToDisplay }) {
  if (hasToDisplay) {
    return <button type="submit" id="create-new-action">Create</button>;
  }
  return null;
}

const NewActionModal = ({ onSubmit, isOpen, onClose }) => {
  const focusInputRef = useRef(null);
  const [formState, setFormState] = useState(initialNewActionModalData);
  const [template, setTemplate] = useState("empty");
  const [createButton, setcreateButton] = useState(false);

  const onOptionChange = e => {
    setTemplate(e.target.value)
    handleInputChange(e)
  }

  useEffect(() => {
    if (isOpen && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
  }, [isOpen]);

  const handleInputChange = (event) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => ({
      ...prevFormData,
      [name]: value,
    }));
    if (name === "actionName") {
      setcreateButton(value !== "");
    }
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    onSubmit(formState);
    setFormState(initialNewActionModalData);
    setcreateButton(false);
  };

  const handleCancel = (event) => {
    event.preventDefault();
    onClose(formState);
    setFormState(initialNewActionModalData);
    setcreateButton(false);
  };

  return (
    <Modal hasCloseBtn={true} isOpen={isOpen} onClose={onClose}>
      <form onSubmit={handleSubmit} onReset={handleCancel}>
        <div className="form-row">
          <label htmlFor="actionName">Action Name</label>
          <input
            ref={focusInputRef}
            type="text"
            id="actionName"
            name="actionName"
            onChange={handleInputChange}
            autoComplete='off'
            required
          />
        </div>
        <div className="form-row" id="templates-list">
          <label htmlFor="templateType">Template Type</label>
          <div className="templates-col">
            <label>
            <input type="radio"
            name="templateType" 
            value="empty"
            id="emptyTemplate"
            checked={template === "empty"}
            onChange={onOptionChange}/>
            <div htmlFor="emptyTemplate">
              <img className="icon" src={empty_template}></img>
              <p> Empty </p>
            </div>
            </label>
          </div>
          <div className="templates-col">
            <label>
            <input type="radio"
              name="templateType" 
              value="action"
              id="actionTemplate"
              checked={template === "action"}
              onChange={onOptionChange}/>
            <div htmlFor="actionTemplate">
              <img className="icon" src={empty_template}></img>
              <p> Action </p>
            </div>
            </label>
          </div>
          <div className="templates-col">
            <label>
            <input type="radio"
              name="templateType" 
              value="io"
              id="ioTemplate"
              checked={template === "io"}
              onChange={onOptionChange}/>
            <div htmlFor="ioTemplate">
              <img className="icon" src={empty_template}></img>
              <p> I/O </p>
            </div>
            </label>
          </div>
        </div>
        <div className="form-row">
          <div className="form-col">
            <CreateButton hasToDisplay={createButton}/>
            <button type="reset" id="cancel-new-action">Cancel</button>
          </div>
        </div>
      </form>
    </Modal>
  );
};

export default NewActionModal;