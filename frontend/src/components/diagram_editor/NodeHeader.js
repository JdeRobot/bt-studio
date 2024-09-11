import React, { useEffect, useState } from "react";
import axios from "axios";
import Menu from "@mui/material/Menu";
import MenuItem from "@mui/material/MenuItem";
import "./NodeHeader.css";

import { ReactComponent as DeleteIcon } from "./img/del_node.svg";
import { ReactComponent as EditActionIcon } from "./img/edit_action.svg";
import { ReactComponent as HelpIcon } from "./img/help.svg";
import { ReactComponent as DownloadIcon } from "./img/download.svg";
import { ReactComponent as RunIcon } from "./img/run.svg";
import { ReactComponent as StopIcon } from "./img/stop.svg";
import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { ReactComponent as ResetIcon } from "./img/reset.svg";

const NodeHeader = ({
  onNodeTypeSelected,
  onDeleteNode,
  onEditAction,
  onDownloadApp,
  onRunApp,
  onResetApp,
  isAppRunning,
  currentProjectname,
  zoomToFit,
}) => {
  const [anchorEl, setAnchorEl] = useState(null);
  const [menuLabel, setMenuLabel] = useState("");

  const handleClick = (event, label) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
    if (label === "Actions") {
      fetchActionList();
    }
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  const handleSelect = (nodeType) => {
    if (onNodeTypeSelected) {
      onNodeTypeSelected(nodeType);
    }
    handleClose();
  };

  // Initialize a state variable to hold the list of action names
  const [actionList, setActionList] = useState([]);

  // Fetch the file list and update actionList
  // TODO: only ask for actions
  const fetchActionList = async () => {
    try {
      const response = await axios.get(
        `/tree_api/get_actions_list?project_name=${currentProjectname}`,
      );
      const files = response.data.actions_list;
      if (Array.isArray(files)) {
        const actions = files.map((file) => file.replace(".py", ""));
        setActionList(actions);
      } else {
        console.error("API response is not an array:", files);
      }
    } catch (error) {
      console.error("Error fetching files:", error);
    }
  };

  const getMenuItems = () => {
    if (menuLabel === "Sequences") {
      return ["Sequence", "ReactiveSequence", "SequenceWithMemory"];
    } else if (menuLabel === "Fallbacks") {
      return ["Fallback", "ReactiveFallback"];
    } else if (menuLabel === "Decorators") {
      return [
        "RetryUntilSuccessful",
        "Inverter",
        "ForceSuccess",
        "ForceFailure",
        "KeepRunningUntilFailure",
        "Repeat",
        "RunOnce",
        "Delay",
      ];
    } else if (menuLabel === "Actions") {
      return actionList; // Use the action names fetched from the API
    } else if (menuLabel === "Port values") {
      return ["Input port value", "Output port value"];
    }
    return [];
  };

  const openInNewTab = (url) => {
    window.open(url, "_blank").focus();
  };

  return (
    <div className="node-header-container">
      <h2>Tree Editor</h2>

      <div className="button-container">
        <button
          className="node-button"
          onClick={(e) => handleClick(e, "Sequences")}
        >
          Sequences
        </button>
        <button
          className="node-button"
          onClick={(e) => handleClick(e, "Fallbacks")}
        >
          Fallbacks
        </button>
        <button
          className="node-button"
          onClick={(e) => handleClick(e, "Decorators")}
        >
          Decorators
        </button>
        <button
          className="node-button"
          onClick={(e) => handleClick(e, "Actions")}
        >
          Actions
        </button>
        <button
          className="node-button"
          onClick={(e) => handleClick(e, "Port values")}
        >
          Port value
        </button>
      </div>

      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={handleClose}>
        {getMenuItems().map((item) => (
          <MenuItem key={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </Menu>

      <div className="action-buttons">
        <button
          id="node-action-delete-button"
          className="node-action-button"
          onClick={onDeleteNode}
          title="Delete"
        >
          <DeleteIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
        <button
          id="node-action-edit-button"
          className="node-action-button"
          onClick={onEditAction}
          title="Edit"
        >
          <EditActionIcon className="icon action-icon" stroke={"var(--icon)"} />
        </button>
        <button
          id="node-action-zoom-button"
          className="node-action-button"
          onClick={zoomToFit}
          title="Zoom To Fit"
        >
          <ZoomToFitIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
        <button
          id="node-action-help-button"
          className="node-action-button"
          onClick={() => {
            openInNewTab(
              "https://github.com/JdeRobot/bt-studio/tree/unibotics-devel/documentation",
            );
          }}
          title="Help"
        >
          <HelpIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
        <button
          className="node-action-button"
          onClick={onDownloadApp}
          title="Download app"
        >
          <DownloadIcon className="icon action-icon" stroke={"var(--icon)"} />
        </button>
        <button
          className="node-action-button"
          onClick={onRunApp}
          title="Run app"
        >
          {isAppRunning ? (
            <StopIcon className="icon action-icon" fill={"var(--icon)"} />
          ) : (
            <RunIcon className="icon action-icon" fill={"var(--icon)"} />
          )}
        </button>
        <button
          className="node-action-button"
          onClick={onResetApp}
          title="Reset app"
        >
          <ResetIcon className="icon action-icon" stroke={"var(--icon)"} />
        </button>
      </div>
    </div>
  );
};

export default NodeHeader;
