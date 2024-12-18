import React, { useMemo, useState, useEffect } from "react";

const Setting = ({ title, children }) => {
  // const [open, setOpen] = useState(false);

  return (
    <div className="bt-setting-setting">
      <label className="bt-setting-setting-title">{title}</label>
      {/* Add settings info */}
      {children}
    </div>
  );
};

export default Setting;
