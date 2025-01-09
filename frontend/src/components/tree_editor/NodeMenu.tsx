import React, { MouseEventHandler, useEffect, useState } from "react";
import "./NodeMenu.css";

import { ReactComponent as DeleteIcon } from "./img/del_node.svg";
import { ReactComponent as SubtreeIcon } from "./img/subtree.svg";
import { ReactComponent as EditActionIcon } from "./img/edit_action.svg";
import { ReactComponent as HelpIcon } from "./img/help.svg";
import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { ReactComponent as EyeOpenIcon } from "./img/eye_open.svg";
import { ReactComponent as EyeClosedIcon } from "./img/eye_closed.svg";
import { ReactComponent as ReturnIcon } from "./img/return.svg";
import { Menu, MenuItem } from "@mui/material";

import {
  createSubtree,
  getSubtreeList,
  getActionsList,
} from "../../api_helper/TreeWrapper";
import { TreeViewType } from "../helper/TreeEditorHelper";
import AddSubtreeModal from "./modals/AddSubtreeModal";

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
      return;
    } else {
      console.error("API response is not an array:", subtreeList);
    }
  } catch (error) {
    if (error instanceof Error) {
      console.error("Error fetching subtrees:", error.message);
    }
  }
  NODE_MENU_ITEMS["Subtrees"] = [];
};

const NodeMenu = ({
  projectName,
  onAddNode,
  onDeleteNode,
  onZoomToFit,
  onEditAction,
  hasSubtrees,
  view,
  changeView,
  setGoBack,
  subTreeName,
  updateFileExplorer
}: {
  projectName: string;
  onAddNode: Function;
  onDeleteNode: MouseEventHandler;
  onZoomToFit: MouseEventHandler;
  onEditAction: MouseEventHandler;
  hasSubtrees: boolean;
  view: TreeViewType;
  changeView: Function;
  setGoBack: Function;
  subTreeName: string;
  updateFileExplorer: Function;
}) => {
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [menuLabel, setMenuLabel] = useState<string>("");
  const [isNewSubtreeModalOpen, setNewSubtreeModalOpen] =
    useState<boolean>(false);

  useEffect(() => {
    const fetchData = async () => {
      await fetchActionList(projectName);
      await fetchSubtreeList(projectName);
    };

    fetchData();
  }, [projectName]);

  const handleClick = (
    event: React.MouseEvent<HTMLButtonElement>,
    label: string
  ) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
    if (label === "Actions") {
      fetchActionList(projectName);
    }
  };

  const handleClose = () => setAnchorEl(null);

  const handleSelect = (nodeName: string) => {
    console.log("Selected: " + nodeName);
    const nodeType = Object.keys(NODE_MENU_ITEMS).find((key) =>
      NODE_MENU_ITEMS[key].includes(nodeName)
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

  const handleCreateSubtree = () => {
    setNewSubtreeModalOpen(true);
  };

  const handleCloseCreateFolder = () => {
    setNewSubtreeModalOpen(false);
    var subtree_input = document.getElementById(
      "subTreeName"
    ) as HTMLInputElement;
    if (subtree_input) {
      subtree_input.value = "";
    }
  };

  const handleCreateFolderSubmit = async (subtreeName: string) => {
    if (subtreeName !== "") {
      try {
        const subtreeId = await createSubtree(subtreeName, projectName);
        console.log("Created subtree:", subtreeId);
        fetchSubtreeList(projectName);
        updateFileExplorer(true);
      } catch (error) {
        console.error("Failed to create subtree:", error);
      }
    }
  };

  return (
    <>
      <div className="bt-node-header-container">
        <div className="bt-button-container">
          {Object.keys(NODE_MENU_ITEMS).map((label) => {
            if (label === "Subtrees" && !hasSubtrees) {
              return null;
            }
            return (
              <button
                key={label}
                className="bt-node-button"
                onClick={(e) => handleClick(e, label)}
              >
                {label}
              </button>
            );
          })}
        </div>

        <Menu
          anchorEl={anchorEl}
          open={Boolean(anchorEl)}
          onClose={handleClose}
        >
          {NODE_MENU_ITEMS[menuLabel]?.map((item) => (
            <MenuItem key={item} onClick={() => handleSelect(item)}>
              {item}
            </MenuItem>
          ))}
        </Menu>

        <div className="bt-action-buttons">
          {hasSubtrees && (
            <button
              id="bt-node-action-subtree-button"
              className="bt-node-action-button"
              onClick={() => {
                handleCreateSubtree();
              }}
              title="Create Subtree"
            >
              <SubtreeIcon
                className="bt-icon bt-action-icon"
                fill={"var(--icon)"}
              />
            </button>
          )}
          <button
            id="bt-node-action-delete-button"
            className="bt-node-action-button"
            onClick={onDeleteNode}
            title="Delete"
          >
            <DeleteIcon
              className="bt-icon bt-action-icon"
              fill={"var(--icon)"}
            />
          </button>
          <button
            id="node-action-edit-button"
            className="bt-node-action-button"
            onClick={onEditAction}
            title="Edit"
          >
            <EditActionIcon
              className="bt-icon bt-action-icon"
              stroke={"var(--icon)"}
            />
          </button>
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
                new URL(
                  "https://github.com/JdeRobot/bt-studio/tree/unibotics-devel/documentation"
                )
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
                  : TreeViewType.Editor
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
      <AddSubtreeModal
        onSubmit={handleCreateFolderSubmit}
        onClose={handleCloseCreateFolder}
        isOpen={isNewSubtreeModalOpen}
        subTreeList={NODE_MENU_ITEMS["Subtrees"]}
      />
    </>
  );
};

export default NodeMenu;
