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
import {
  subscribe,
  TreeViewType,
  unsubscribe,
} from "../helper/TreeEditorHelper";
import AddSubtreeModal from "./modals/AddSubtreeModal";
import { useError } from "../error_popup/ErrorModal";

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
  updateFileExplorer,
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
  const { error } = useError();

  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [menuLabel, setMenuLabel] = useState<string>();
  const [menuList, setMenuList] = useState<string[]>([]);
  const [actionsList, updateActionsList] = useState<string[]>([]);
  const [subtreesList, updateSubtreesList] = useState<string[]>([]);
  const [isNewSubtreeModalOpen, setNewSubtreeModalOpen] =
    useState<boolean>(false);

  const fetchActionList = async () => {
    console.log("Fetching actions...");
    try {
      const files = await getActionsList(projectName);
      const actions = files.map((file: string) => file.replace(".py", ""));
      updateActionsList(actions);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error fetching files:", e.message);
        // error("Failed to fetch files: " + e.message);
      }
      updateActionsList([]);
    }
  };

  const fetchSubtreeList = async () => {
    console.log("Fetching subtrees...");
    try {
      const subtreeList = await getSubtreeList(projectName);
      console.log("Subtree list:", subtreeList);
      updateSubtreesList(subtreeList);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error fetching subtrees:", e.message);
        // error("Failed to fetch subtrees: " + e.message);
      }
      updateSubtreesList([]);
    }
  };

  useEffect(() => {
    subscribe("updateActionList", fetchActionList);
    subscribe("updateSubtreeList", fetchSubtreeList);

    return () => {
      unsubscribe("updateActionList", () => {});
      unsubscribe("updateSubtreeList", () => {});
    };
  }, []);

  useEffect(() => {
    const fetchData = async () => {
      await fetchActionList();
      await fetchSubtreeList();
    };

    fetchData();
  }, [projectName]);

  const handleClick = (
    event: React.MouseEvent<HTMLButtonElement>,
    label: string,
  ) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
    if (label === "Actions") {
      setMenuList(actionsList);
    } else if (label === "Subtrees") {
      setMenuList(subtreesList);
    } else {
      setMenuList(NODE_MENU_ITEMS[label]);
    }
  };

  const handleClose = () => setAnchorEl(null);

  const handleSelect = (nodeName: string) => {
    console.log("Selected: " + nodeName);
    var nodeType: string | undefined;

    if (menuLabel === "Actions") {
      nodeType = "Actions";
    } else if (menuLabel === "Subtrees") {
      nodeType = "Subtrees";
    } else {
      nodeType = Object.keys(NODE_MENU_ITEMS).find((key) =>
        NODE_MENU_ITEMS[key].includes(nodeName),
      );
    }

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
      "subTreeName",
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
        fetchSubtreeList();
        updateFileExplorer(true);
      } catch (e) {
        if (e instanceof Error) {
          console.error("Failed to create subtree: " + e.message);
          error("Failed to create subtree: " + e.message);
        }
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
                id={label}
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
          {menuList.map((item) => (
            <MenuItem key={item} id={item} onClick={() => handleSelect(item)}>
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
                  "https://jderobot.github.io/bt-studio/documentation/",
                ),
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
