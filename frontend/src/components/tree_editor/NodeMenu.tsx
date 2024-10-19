import React, { MouseEventHandler, useEffect, useState } from "react";
import "./NodeMenu.css";
import axios from "axios";

import { ReactComponent as DeleteIcon } from "./img/del_node.svg";
import { ReactComponent as SubtreeIcon } from "./img/subtree.svg";
import { ReactComponent as EditActionIcon } from "./img/edit_action.svg";
import { ReactComponent as HelpIcon } from "./img/help.svg";
import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { ReactComponent as ReturnIcon } from "./img/return.svg";
import { Menu, MenuItem } from "@mui/material";

import {
  createSubtree,
  getSubtreeList,
  getActionsList,
} from "../../api_helper/TreeWrapper";

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
  Subtrees: [],
  "Port values": ["Input port value", "Output port value"],
};

const fetchActionList = async (project_name: string) => {
  try {
    const files = await getActionsList(project_name);
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

const fetchSubtreeList = async (project_name: string) => {
  console.log("Fetching subtrees...");
  try {
    const subtreeList = await getSubtreeList(project_name);
    console.log("Subtree list:", subtreeList);
    if (Array.isArray(subtreeList)) {
      NODE_MENU_ITEMS["Subtrees"] = subtreeList;
    } else {
      console.error("API response is not an array:", subtreeList);
    }
  } catch (error) {
    if (error instanceof Error) {
      console.error("Error fetching subtrees:", error.message);
    }
  }
};

const NodeMenu = ({
  projectName,
  onAddNode,
  onDeleteNode,
  onZoomToFit,
  onEditAction,
  hasSubtrees,
  setGoBack,
}: {
  projectName: string;
  onAddNode: Function;
  onDeleteNode: MouseEventHandler;
  onZoomToFit: MouseEventHandler;
  onEditAction: MouseEventHandler;
  hasSubtrees: boolean;
  setGoBack: Function;
}) => {
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [menuLabel, setMenuLabel] = useState<string>("");

  useEffect(() => {
    const fetchData = async () => {
      await fetchActionList(projectName);
      await fetchSubtreeList(projectName);
    };

    fetchData();
  }, [projectName]);

  const handleClick = (
    event: React.MouseEvent<HTMLButtonElement>,
    label: string,
  ) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
  };

  const handleClose = () => setAnchorEl(null);

  const handleSelect = (nodeName: string) => {
    console.log("Selected: " + nodeName);
    const nodeType = Object.keys(NODE_MENU_ITEMS).find((key) =>
      NODE_MENU_ITEMS[key].includes(nodeName),
    );
    if (nodeType) {
      console.log("Node Type: " + nodeType);
      onAddNode(nodeType, nodeName);
    } else {
      console.log("Unknown node type");
    }
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

  const onCreateSubtree = async () => {
    try {
      const subtreeName = prompt("Enter subtree name:");
      if (subtreeName) {
        const subtreeId = await createSubtree(subtreeName, projectName);
        console.log("Created subtree:", subtreeId);
        fetchSubtreeList(projectName);
      }
    } catch (error) {
      console.error("Failed to create subtree:", error);
    }
  };

  return (
    <div className="node-header-container">
      <div className="button-container">
        {Object.keys(NODE_MENU_ITEMS).map((label) => {
          if (label === "Subtrees" && !hasSubtrees) {
            return null;
          }
          return (
            <button
              key={label}
              className="node-button"
              onClick={(e) => handleClick(e, label)}
            >
              {label}
            </button>
          );
        })}
      </div>

      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={handleClose}>
        {NODE_MENU_ITEMS[menuLabel]?.map((item) => (
          <MenuItem key={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </Menu>

      <div className="action-buttons">
        {hasSubtrees && (
          <button
            id="node-action-subtree-button"
            className="node-action-button"
            onClick={() => {
              onCreateSubtree();
            }}
            title="Create Subtree"
          >
            <SubtreeIcon className="icon action-icon" fill={"var(--icon)"} />
          </button>
        )}
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
                "https://github.com/JdeRobot/bt-studio/tree/unibotics-devel/documentation",
              ),
            );
          }}
          title="Help"
        >
          <HelpIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
        <button
          id="node-action-back-button"
          className="node-action-button"
          onClick={() => setGoBack(true)}
          title="Go Back"
        >
          <ReturnIcon className="icon action-icon" fill={"var(--icon)"} />
        </button>
      </div>
      <h3 className="subtree-name">Placeholder for subtree name</h3>
    </div>
  );
};

export default NodeMenu;
