import React, { useMemo, useState, useEffect } from "react";

import "./Checkbox.css";

const Checkbox = ({ setting }) => {
  return (
    <div className="setting-checkbox-wrapper">
      <input
        type="checkbox"
        checked={setting.value}
        onChange={() => setting.setter(!setting.value)}
      />
    </div>
  );
};

export default Checkbox;
