import React, {
  MouseEventHandler,
  useContext,
  useEffect,
  useState,
} from "react";
import "./NodeMenu.css";

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
import { publish, subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import AddSubtreeModal from "./modals/AddSubtreeModal";
import { useError } from "../error_popup/ErrorModal";
import { OptionsContext } from "../options/Options";

export const BTSelectorButtons = ({ project }: { project: string }) => {
  const { error } = useError();

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

  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [menuLabel, setMenuLabel] = useState<string>();
  const [menuList, setMenuList] = useState<string[]>([]);
  const [actionsList, updateActionsList] = useState<string[]>([]);
  const [subtreesList, updateSubtreesList] = useState<string[]>([]);

  const fetchActionList = async () => {
    console.log("Fetching actions...");
    try {
      const files = await getActionsList(project);
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
      const subtreeList = await getSubtreeList(project);
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
  }, [project]);

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
      publish("addBTNode", { type: nodeType, name: nodeName });
    } else {
      console.log("Unknown node type");
    }
    handleClose();
  };

  return (
    <>
      <div className="bt-node-header-container">
        <div className="bt-button-container">
          {Object.keys(NODE_MENU_ITEMS).map((label) => {
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
      </div>
    </>
  );
};

export const AddSubtreeButton = ({ project }: { project: string }) => {
  const { error } = useError();

  const [subtreesList, updateSubtreesList] = useState<string[]>([]);
  const [isNewSubtreeModalOpen, setNewSubtreeModalOpen] =
    useState<boolean>(false);

  const fetchSubtreeList = async () => {
    console.log("Fetching subtrees...");
    try {
      const subtreeList = await getSubtreeList(project);
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
    subscribe("updateSubtreeList", fetchSubtreeList);

    return () => {
      unsubscribe("updateSubtreeList", () => {});
    };
  }, []);

  useEffect(() => {
    const fetchData = async () => {
      await fetchSubtreeList();
    };

    fetchData();
  }, [project]);

  const handleCreateSubtree = () => {
    setNewSubtreeModalOpen(true);
  };

  const handleCloseCreateSubtree = () => {
    setNewSubtreeModalOpen(false);
    var subtree_input = document.getElementById(
      "subTreeName",
    ) as HTMLInputElement;
    if (subtree_input) {
      subtree_input.value = "";
    }
  };

  const handleCreateSubtreeSubmit = async (subtreeName: string) => {
    if (subtreeName !== "") {
      try {
        const subtreeId = await createSubtree(subtreeName, project);
        console.log("Created subtree:", subtreeId);
        fetchSubtreeList();
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
        <div className="bt-action-buttons">
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
        </div>
      </div>
      <AddSubtreeModal
        onSubmit={handleCreateSubtreeSubmit}
        onClose={handleCloseCreateSubtree}
        isOpen={isNewSubtreeModalOpen}
        subTreeList={subtreesList}
      />
    </>
  );
};

export const OtherButtons = ({ project }: { project: string }) => {
  const settings = useContext(OptionsContext);

  const [btOrder, setBtOrder] = useState(settings.btOrder.default_value);

  useEffect(() => {
    setBtOrder(settings.btOrder.value);
  }, [settings.btOrder.value]);

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
            id="bt-node-action-delete-button"
            className="bt-node-action-button"
            onClick={() => publish("BTEditorDeleteCurrent")}
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
            onClick={() => publish("BTEditorEditCurrent")}
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
            onClick={() => publish("BTEditorHomeZoom")}
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
            id="bt-node-action-help-button"
            className="bt-node-action-button"
            title={"BT Order: " + btOrder}
          >
            <svg
              className="w-6 h-6 text-gray-800 dark:text-white"
              aria-hidden="true"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              fill="none"
              viewBox="0 0 24 24"
            >
              {btOrder === "bottom-to-top" ? (
                <path
                  stroke="var(--icon)"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth="2"
                  d="M12 6v13m0-13 4 4m-4-4-4 4"
                />
              ) : (
                <path
                  stroke="var(--icon)"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth="2"
                  d="M12 19V5m0 14-4-4m4 4 4-4"
                />
              )}
            </svg>
          </button>
        </div>
      </div>
    </>
  );
};
