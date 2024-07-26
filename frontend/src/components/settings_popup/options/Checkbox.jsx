import React, { useMemo, useState, useEffect } from 'react';

const Checkbox = ({value, setValue}) => {
  const [open, setOpen] = useState(false);

  return (
    // <div className="settings-checkbox">
    //   <button onClick={(e) => { e.preventDefault(); setOpen(!open)}}>{value}</button>
    //   {open &&
    //     <ul className="settings-dropdown-list">
    //       {possibleValues.map((name, index) => (
    //         <li key={name} className="settings-dropdown-item">
    //           <button onClick={(e) => changeValue(e,name)}>{name}</button>
    //         </li>
    //       ))}
    //     </ul>
    //   }
    // </div>
    <input type="checkbox" className='setting-setting-checkbox' checked={value} onChange={() => setValue(!value)}/>
  );
};

export default Checkbox;