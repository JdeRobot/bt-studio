import React, { useMemo, useState, useEffect } from 'react';

const Section = ({title, children}) => {
  // const [open, setOpen] = useState(false);

  return (
    <div className='setting-section'>
      <label className='setting-section-title'>{title}</label>
      {children}
    </div>
  );
};

export default Section;