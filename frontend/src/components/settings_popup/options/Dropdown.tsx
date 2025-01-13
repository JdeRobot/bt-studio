import { useRef, useState } from "react";

import "./Dropdown.css";
import { SettingData } from "../../options/Options";

const Dropdown = ({
  setting,
  possibleValues,
}: {
  setting: SettingData<any>;
  possibleValues: any[];
}) => {
  const [open, setOpen] = useState<boolean>(false);
  const dropdown = useRef<HTMLDivElement>(null);

  const changeValue = (e: any, value: any) => {
    e.preventDefault();
    setting.setter(value);
  };

  const closeOpenMenus = (e: any) => {
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
              onClick={(e: any) => changeValue(e, name)}
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
