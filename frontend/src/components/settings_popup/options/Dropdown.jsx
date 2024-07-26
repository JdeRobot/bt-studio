import React, { useMemo, useState, useEffect } from 'react';

const Dropdown = ({value, setValue, possibleValues }) => {
  const [open, setOpen] = useState(false);

  const changeValue = (e, value) => {
    e.preventDefault()
    setValue(value)
  }

  return (
    <div className="settings-dropdown">
      <button onClick={(e) => { e.preventDefault(); setOpen(!open)}}>{value}</button>
      {open &&
        <ul className="settings-dropdown-list">
          {possibleValues.map((name, index) => (
            <li key={name} className="settings-dropdown-item">
              <button onClick={(e) => changeValue(e,name)}>{name}</button>
            </li>
          ))}
        </ul>
      }
    </div>
  );
};

export default Dropdown;