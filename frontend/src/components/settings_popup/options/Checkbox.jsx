import React, { useMemo, useState, useEffect } from 'react';

import './Checkbox.css';

const Checkbox = ({value, setValue}) => {

  return (
    <div className="setting-checkbox-wrapper">
      <input type="checkbox" checked={value} onChange={() => setValue(!value)}/>
    </div>
  );
};

export default Checkbox;