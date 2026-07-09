import React from "react";
import "./Checkbox.css";
import { SettingData } from "BtContexts/ProjectSettingsContext";

const Checkbox = ({ setting }: { setting: SettingData<any> }) => {
  return (
    <div className="bt-setting-checkbox-wrapper">
      <input
        type="checkbox"
        checked={setting.value}
        onChange={() => setting.setter(!setting.value)}
      />
    </div>
  );
};

export default Checkbox;
