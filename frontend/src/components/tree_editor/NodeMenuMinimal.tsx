import React, { MouseEventHandler } from "react";
import "./NodeMenu.css";

import { ReactComponent as HelpIcon } from "./img/help.svg";
import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { ReactComponent as EyeOpenIcon } from "./img/eye_open.svg";
import { ReactComponent as EyeClosedIcon } from "./img/eye_closed.svg";
import { ReactComponent as ReturnIcon } from "./img/return.svg";

import { TreeViewType } from "../helper/TreeEditorHelper";

const NodeMenuMinimal = ({
  onZoomToFit,
  view,
  changeView,
  setGoBack,
  subTreeName,
}: {
  onZoomToFit: MouseEventHandler;
  view: TreeViewType;
  changeView: Function;
  setGoBack: Function;
  subTreeName: string;
}) => {
  const openInNewTab = (url: URL) => {
    const newWindow = window.open(url, "_blank");
    if (newWindow) {
      newWindow.focus();
    } else {
      console.error("Failed to open new tab/window.");
    }
  };

  return (
    <>
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
            id="bt-node-action-help-button"
            className="bt-node-action-button"
            onClick={() => {
              openInNewTab(
                new URL("https://jderobot.github.io/bt-studio/documentation/"),
              );
            }}
            title="Help"
          >
            <HelpIcon className="bt-icon bt-action-icon" fill={"var(--icon)"} />
          </button>
          <button
            id="bt-node-change-view-button"
            className="bt-node-action-button"
            onClick={() =>
              changeView(
                view === TreeViewType.Editor
                  ? TreeViewType.Visualizer
                  : TreeViewType.Editor,
              )
            }
            title="Change view"
          >
            {view === TreeViewType.Editor ? (
              <EyeOpenIcon className="bt-header-icon" stroke={"var(--icon)"} />
            ) : (
              <EyeClosedIcon
                className="bt-header-icon"
                stroke={"var(--icon)"}
              />
            )}
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
        <h2 className="bt-subtree-name">{subTreeName}</h2>
      </div>
    </>
  );
};

export default NodeMenuMinimal;
