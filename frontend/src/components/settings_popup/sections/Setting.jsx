import React, { useMemo, useState, useEffect } from 'react';

const Setting = ({title, children}) => {
  // const [open, setOpen] = useState(false);

  return (
    <div className='setting-setting'>
      <label className='setting-setting-title'>{title}</label>
      {/* Add settings info */}
      {children}
    </div>
  );
};

export default Setting;