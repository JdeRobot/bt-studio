import React, { useEffect, useState } from "react";

import { ReactComponent as ClosedArrowIcon } from "./img/arrowSide.svg";
import { ReactComponent as OpenArrowIcon } from "./img/arrowDown.svg";
import { ReactComponent as ClosedFolderIcon } from "./img/closedFolder.svg";
import { ReactComponent as OpenFolderIcon } from "./img/openFolder.svg";
import { ReactComponent as BaseFileIcon } from "./img/file.svg";
import { ReactComponent as ActionFileIcon } from "./img/file-action.svg";

function FileIcon({ is_dir, is_collapsed, name, group }) {
  var returnVal = (
    <>
      <BaseFileIcon className="bt-arrow-icon" fill={"var(--icon)"} />
    </>
  );

  if (is_dir) {
    if (is_collapsed) {
      return (
        <>
          <ClosedArrowIcon className="bt-arrow-icon" stroke={"var(--icon)"} />
          <ClosedFolderIcon className="bt-arrow-icon" fill={"var(--icon)"} />
        </>
      );
    } else {
      return (
        <>
          <OpenArrowIcon className="bt-arrow-icon" stroke={"var(--icon)"} />
          <OpenFolderIcon className="bt-arrow-icon" fill={"var(--icon)"} />
        </>
      );
    }
  }

  switch (group) {
    case "Action":
      returnVal = (
        <>
          <ActionFileIcon className="bt-arrow-icon" fill={"var(--icon)"} />
        </>
      );
      break;

    default:
      break;
  }
  return returnVal;
}

export default FileIcon;
