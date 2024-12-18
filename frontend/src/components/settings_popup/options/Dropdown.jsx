import React, { useRef, useState, useEffect } from "react";

import "./Dropdown.css";

const Dropdown = ({ setting, possibleValues }) => {
  const [open, setOpen] = useState(false);
  const dropdown = useRef(null);

  const changeValue = (e, value) => {
    e.preventDefault();
    setting.setter(value);
  };

  const closeOpenMenus = (e) => {
    if (open && !dropdown.current?.contains(e.target)) {
      setOpen(false);
    }
  };

  document.addEventListener("mousedown", closeOpenMenus);

  return (
    <div className="bt-settings-dropdown" ref={dropdown}>
      <button
        className="bt-settings-dropdown-preview"
        onClick={(e) => {
          e.preventDefault();
          setOpen(!open);
        }}
      >
        {setting.value}
      </button>
      {open && (
        <div className="bt-settings-dropdown-list">
          {possibleValues.map((name, index) => (
            <button
              className="bt-settings-dropdown-item"
              onClick={(e) => changeValue(e, name)}
            >
              {name}
            </button>
          ))}
        </div>
      )}
    </div>
  );
};

export default Dropdown;
