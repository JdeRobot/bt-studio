import React from "react";
import { MouseEventHandler, MutableRefObject } from "react";

import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { ReactComponent as ReturnIcon } from "./img/return.svg";

const TreeMonitorMenu = ({
  onZoomToFit,
  setGoBack,
  subTreeName,
}: {
  onZoomToFit: MouseEventHandler;
  setGoBack: (a: boolean) => void;
  subTreeName: MutableRefObject<string>;
}) => {
  return (
    <div className="bt-node-header-container">
      <div className="bt-action-buttons">
        <button
          id="bt-node-action-zoom-button"
          className="bt-node-action-button"
          onClick={onZoomToFit}
          title="Zoom To Fit"
        >
          <ZoomToFitIcon
            className="bt-icon bt-action-icon"
            fill={"var(--icon)"}
          />
        </button>
        <button
          id="node-action-back-button"
          className="bt-node-action-button"
          onClick={() => setGoBack(true)}
          title="Go Back"
        >
          <ReturnIcon className="bt-header-icon" fill={"var(--icon)"} />
        </button>
      </div>
      <h2 className="bt-subtree-name">{subTreeName.current}</h2>
    </div>
  );
};

export default TreeMonitorMenu;
