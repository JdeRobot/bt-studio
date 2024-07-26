import React, { useMemo, useState, useEffect } from 'react';

const SubSection = ({title, children}) => {
  // const [open, setOpen] = useState(false);

  return (
    <div className='setting-subsection'>
      <label className='setting-subsection-title'>{title}</label>
      {children}
    </div>
  );
};

export default SubSection;