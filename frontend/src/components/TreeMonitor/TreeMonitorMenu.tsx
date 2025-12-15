import React from "react";
import { MouseEventHandler, MutableRefObject } from "react";

import { ReactComponent as ReturnIcon } from "./img/return.svg";
import { UndoIcon, ZoomIcon } from "BtIcons";
import { useBtTheme } from "BtContexts/BtThemeContext";

const TreeMonitorMenu = ({
  onZoomToFit,
  setGoBack,
  subTreeName,
}: {
  onZoomToFit: MouseEventHandler;
  setGoBack: (a: boolean) => void;
  subTreeName: MutableRefObject<string>;
}) => {
  const theme = useBtTheme();
  
  return (
    <div className="bt-node-header-container">
      <div className="bt-action-buttons">
        <button
          id="bt-node-action-zoom-button"
          className="bt-node-action-button"
          onClick={onZoomToFit}
          title="Zoom To Fit"
        >
          <ZoomIcon
            className="bt-icon bt-action-icon"
            htmlColor={theme.palette.text}
          />
        </button>
        <button
          id="node-action-back-button"
          className="bt-node-action-button"
          onClick={() => setGoBack(true)}
          title="Go Back"
        >
          <UndoIcon className="bt-header-icon" htmlColor={theme.palette.text} />
        </button>
      </div>
      <h2 className="bt-subtree-name">{subTreeName.current}</h2>
    </div>
  );
};

export default TreeMonitorMenu;
