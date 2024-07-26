import React, { useRef, useState, useEffect } from 'react';

import './Dropdown.css';

const Dropdown = ({value, setValue, possibleValues }) => {
  const [open, setOpen] = useState(false);
  const dropdown = useRef(null)

  const changeValue = (e, value) => {
    e.preventDefault()
    setValue(value)
  }

  const closeOpenMenus = (e)=>{
    if(open && !dropdown.current?.contains(e.target)) {
      setOpen(false)
    }
  }

  document.addEventListener('mousedown',closeOpenMenus)

  return (
    <div className="settings-dropdown" ref={dropdown}>
      <button className="settings-dropdown-preview" onClick={(e) => { e.preventDefault(); setOpen(!open)}}>{value}</button>
      {open &&
        <div className="settings-dropdown-list">
          {possibleValues.map((name, index) => (
            <button  className="settings-dropdown-item" onClick={(e) => changeValue(e,name)}>{name}</button>
          ))}
        </div>
      }
    </div>
  );
};

export default Dropdown;