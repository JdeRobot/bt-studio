import React, { MouseEventHandler, useEffect, useState } from "react";
import Menu from "@mui/material/Menu";
import MenuItem from "@mui/material/MenuItem";
import "./NodeHeader.css";
import axios from "axios";

import { ReactComponent as DeleteIcon } from "./img/del_node.svg";
import { ReactComponent as EditActionIcon } from "./img/edit_action.svg";
import { ReactComponent as HelpIcon } from "./img/help.svg";
import { ReactComponent as DownloadIcon } from "./img/download.svg";
import { ReactComponent as RunIcon } from "./img/run.svg";
import { ReactComponent as StopIcon } from "./img/stop.svg";
import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { ReactComponent as ResetIcon } from "./img/reset.svg";

var NODE_MENU_ITEMS: Record<string, string[]> = {
  Sequences: ["Sequence", "ReactiveSequence", "SequenceWithMemory"],
  Fallbacks: ["Fallback", "ReactiveFallback"],
  Decorators: [
    "RetryUntilSuccessful",
    "Inverter",
    "ForceSuccess",
    "ForceFailure",
    "KeepRunningUntilFailure",
    "Repeat",
    "RunOnce",
    "Delay",
  ],
  Actions: [],
  "Port values": ["Input port value", "Output port value"],
  "Sub Tree": ["Sub Tree"],
};

const fetchActionList = async (project_name: string) => {
  try {
    const response = await axios.get(
      `/tree_api/get_file_list?project_name=${project_name}`
    );
    const files = response.data.file_list;
    if (Array.isArray(files)) {
      const actions = files.map((file) => file.replace(".py", ""));
      NODE_MENU_ITEMS["Actions"] = actions;
    } else {
      console.error("API response is not an array:", files);
    }
  } catch (error) {
    console.error("Error fetching files:", error);
  }
};

const NodeMenu = ({
  projectName,
  onAddNode,
  onDeleteNode,
  onZoomToFit,
  onEditAction,
}: {
  projectName: string;
  onAddNode: Function;
  onDeleteNode: MouseEventHandler;
  onZoomToFit: MouseEventHandler;
  onEditAction: MouseEventHandler;
}) => {
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [menuLabel, setMenuLabel] = useState<string>("");

  useEffect(() => {
    fetchActionList(projectName);
  }, [projectName]);

  const handleClick = (
    event: React.MouseEvent<HTMLButtonElement>,
    label: string
  ) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
  };

  const handleClose = () => setAnchorEl(null);

  const handleSelect = (nodeName: string) => {
    console.log("Selected: " + nodeName);
    onAddNode(nodeName);
    handleClose();
  };

  const openInNewTab = (url: URL) => {
    const newWindow = window.open(url, "_blank");
    if (newWindow) {
      newWindow.focus();
    } else {
      console.error("Failed to open new tab/window.");
    }
  };

  return (
    <div className="node-header-container">
      <h2>Tree Editor</h2>

      <div className="button-container">
        {Object.keys(NODE_MENU_ITEMS).map((label) => (
          <button
            key={label}
            className="node-button"
            onClick={(e) => handleClick(e, label)}
          >
            {label}
          </button>
        ))}
      </div>

      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={handleClose}>
        {NODE_MENU_ITEMS[menuLabel]?.map((item) => (
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
          onClick={onZoomToFit}
          title="Zoom To Fit"
        >
          <ZoomToFitIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
        <button
          id="node-action-help-button"
          className="node-action-button"
          onClick={() => {
            openInNewTab(
              new URL(
                "https://github.com/JdeRobot/bt-studio/tree/unibotics-devel/documentation"
              )
            );
          }}
          title="Help"
        >
          <HelpIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
      </div>
    </div>
  );
};

export default NodeMenu;
