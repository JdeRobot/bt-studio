import React, { useContext, useEffect, useState } from "react";
import "./NodeMenu.css";

import { ReactComponent as DeleteIcon } from "./img/del_node.svg";
import { ReactComponent as SubtreeIcon } from "./img/subtree.svg";
import { ReactComponent as EditActionIcon } from "./img/edit_action.svg";
import { ReactComponent as HelpIcon } from "./img/help.svg";
import { ReactComponent as ZoomToFitIcon } from "./img/zoom_to_fit.svg";
import { Menu, MenuItem } from "@mui/material";
import { ImportIcon } from "../icons";

import {
  createSubtree,
  getSubtreeList,
  getActionsList,
} from "../../api_helper/TreeWrapper";
import { publish, subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import AddSubtreeModal from "./modals/AddSubtreeModal";
import { useError } from "jderobot-ide-interface";
import { OptionsContext } from "../options/Options";
import {
  MenuButton,
  MenuButtonLabel,
  MenuButtonStroke,
} from "jderobot-ide-interface";
import ImportSubtreeModal from "./modals/ImportSubtreeModal";

export const BTSelectorButtons = ({ project }: { project: string }) => {
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
  };

  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [menuLabel, setMenuLabel] = useState<string>();
  const [menuList, setMenuList] = useState<string[]>([]);
  const [actionsList, updateActionsList] = useState<string[]>([]);

  const fetchActionList = async () => {
    console.log("Fetching actions...");
    try {
      const files = await getActionsList(project);
      const actions = files.map((file: string) => file.replace(".py", ""));
      updateActionsList(actions);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error fetching files:", e.message);
      }
      updateActionsList([]);
    }
  };

  useEffect(() => {
    subscribe("updateActionList", fetchActionList);

    return () => {
      unsubscribe("updateActionList", () => {});
    };
  }, []);

  useEffect(() => {
    const fetchData = async () => {
      await fetchActionList();
    };

    fetchData();
  }, [project]);

  const handleClick = (
    event: React.MouseEvent<HTMLButtonElement>,
    label: string,
  ) => {
    console.log(event, label);
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
    if (label === "Actions") {
      setMenuList(actionsList);
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
      {Object.keys(NODE_MENU_ITEMS).map((label) => {
        return (
          <MenuButtonLabel
            key={label}
            id={label}
            title={label}
            onClick={(e: any) => handleClick(e, label)}
          >
            {label}
          </MenuButtonLabel>
        );
      })}

      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={handleClose}>
        {menuList.map((item) => (
          <MenuItem key={item} id={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </Menu>
    </>
  );
};

export const AddSubtreeButton = ({ project }: { project: string }) => {
  const { error } = useError();

  const [subtreesList, updateSubtreesList] = useState<string[]>([]);
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const [isNewSubtreeModalOpen, setNewSubtreeModalOpen] =
    useState<boolean>(false);
  const [importSubtreeModalOpen, setImportSubtreeModalOpen] =
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

  const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleClose = () => setAnchorEl(null);

  const handleSelect = (nodeName: string) => {
    publish("addBTNode", { type: "Subtrees", name: nodeName });
    handleClose();
  };

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

  const handleImportSubtree = () => {
    setImportSubtreeModalOpen(true);
  };

  const handleCloseImportSubtree = () => {
    setImportSubtreeModalOpen(false);
    var subtree_input = document.getElementById(
      "subtreeName",
    ) as HTMLInputElement;
    if (subtree_input) {
      subtree_input.value = "";
    }
  };

  return (
    <>
      <MenuButtonLabel
        key={"Subtrees"}
        id={"Subtrees"}
        title={"Subtrees"}
        onClick={(e: React.MouseEvent<HTMLButtonElement, MouseEvent>) =>
          handleClick(e)
        }
      >
        {"Subtrees"}
      </MenuButtonLabel>

      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={handleClose}>
        {subtreesList.map((item) => (
          <MenuItem key={item} id={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </Menu>
      <MenuButton
        id="bt-node-action-subtree-button"
        onClick={() => {
          handleCreateSubtree();
        }}
        title="Create Subtree"
      >
        <SubtreeIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButtonStroke
        id="import-subtree-button"
        onClick={() => {
          handleImportSubtree();
        }}
        title="Import Subtree"
      >
        <ImportIcon className="bt-icon bt-action-icon" />
      </MenuButtonStroke>
      <ImportSubtreeModal
        project={project}
        onSubmit={handleCloseImportSubtree}
        onClose={handleCloseImportSubtree}
        isOpen={importSubtreeModalOpen}
        subTreeList={subtreesList}
      />
      <AddSubtreeModal
        project={project}
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
      <MenuButton
        id="bt-node-action-delete-button"
        onClick={() => publish("BTEditorDeleteCurrent")}
        title="Delete"
      >
        <DeleteIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButtonStroke
        id="node-action-edit-button"
        onClick={() => publish("BTEditorEditCurrent")}
        title="Edit"
      >
        <EditActionIcon className="bt-icon bt-action-icon" />
      </MenuButtonStroke>
      <MenuButton
        id="bt-node-action-zoom-button"
        onClick={() => publish("BTEditorHomeZoom")}
        title="Zoom To Fit"
      >
        <ZoomToFitIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButton
        id="bt-node-action-help-button"
        onClick={() => {
          openInNewTab(
            new URL("https://jderobot.github.io/bt-studio/documentation/"),
          );
        }}
        title="Help"
      >
        <HelpIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButtonStroke
        id="bt-node-action-help-button"
        title={"BT Order: " + btOrder}
        onClick={() => {
          if (settings.btOrder.value === "bottom-to-top") {
            settings.btOrder.setter("top-to-bottom");
          } else {
            settings.btOrder.setter("bottom-to-top");
          }
        }}
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
      </MenuButtonStroke>
    </>
  );
};
