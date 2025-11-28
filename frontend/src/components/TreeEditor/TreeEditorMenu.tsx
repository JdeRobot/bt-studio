import React, { useEffect, useState } from "react";
import "./NodeMenu.css";
import { Menu, MenuItem, styled } from "@mui/material";
import { ImportIcon } from "../icons";

import {
  createSubtree,
  getSubtreeList,
  getActionsList,
} from "BtApi/TreeWrapper";
import { publish, subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import AddSubtreeModal from "./modals/AddSubtreeModal";
import { useError } from "jderobot-ide-interface";
import {
  MenuButton,
  MenuButtonLabel,
  MenuButtonStroke,
} from "jderobot-ide-interface";
import ImportSubtreeModal from "./modals/ImportSubtreeModal";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  DeleteIcon,
  DownArrowIcon,
  EditIcon,
  TreeIcon,
  UpArrowIcon,
  ZoomIcon,
} from "BtIcons";
import { useProjectSettings } from "BtContexts/ProjectSettingsContext";

const StyledMenu = styled(Menu)(
  ({
    bgColor,
    textColor,
    hoverColor,
    roundness,
  }: {
    bgColor: string;
    textColor: string;
    checkboxColor: string;
    hoverColor: string;
    roundness: number;
  }) => ({
    "& .MuiPaper-root": {
      border: "1px solid black",
      borderRadius: roundness + "px",
      backgroundColor: bgColor,
      "& .MuiMenuItem-root": {
        color: textColor,
        "&:hover": {
          backgroundColor: hoverColor,
        },
        "& .Mui-disabled": {
          "& .MuiSvgIcon-root": {
            opacity: "30%",
          },
        },
      },
    },
  }),
);

export const BTSelectorButtons = ({ project }: { project: string }) => {
  const NODE_MENU_ITEMS: Record<string, string[]> = {
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
  const theme = useBtTheme();

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
    let nodeType: string | undefined;

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

      <StyledMenu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleClose}
        bgColor={theme.palette.primary!}
        hoverColor={theme.palette.secondary!}
        textColor={theme.palette.text!}
        checkboxColor={theme.palette.text!}
        roundness={theme.roundness!}
      >
        {menuList.map((item) => (
          <MenuItem key={item} id={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </StyledMenu>
    </>
  );
};

export const AddSubtreeButton = ({ project }: { project: string }) => {
  const theme = useBtTheme();
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
    const subtree_input = document.getElementById(
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
    const subtree_input = document.getElementById(
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

      <StyledMenu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleClose}
        bgColor={theme.palette.primary!}
        hoverColor={theme.palette.secondary!}
        textColor={theme.palette.text!}
        checkboxColor={theme.palette.text!}
        roundness={theme.roundness!}
      >
        {subtreesList.map((item) => (
          <MenuItem key={item} id={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </StyledMenu>
      <MenuButton
        id="bt-node-action-subtree-button"
        onClick={() => {
          handleCreateSubtree();
        }}
        title="Create Subtree"
      >
        <TreeIcon className="bt-icon bt-action-icon" />
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

export const OtherButtons = () => {
  const settings = useProjectSettings();

  const [btOrder, setBtOrder] = useState(settings.btOrder.default_value);

  useEffect(() => {
    setBtOrder(settings.btOrder.value);
  }, [settings.btOrder.value]);

  return (
    <>
      <MenuButton
        id="bt-node-action-delete-button"
        onClick={() => publish("BTEditorDeleteCurrent")}
        title="Delete"
      >
        <DeleteIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButton
        id="node-action-edit-button"
        onClick={() => publish("BTEditorEditCurrent")}
        title="Edit"
      >
        <EditIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButton
        id="bt-node-action-zoom-button"
        onClick={() => publish("BTEditorHomeZoom")}
        title="Zoom To Fit"
      >
        <ZoomIcon className="bt-icon bt-action-icon" />
      </MenuButton>
      <MenuButton
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
        {btOrder === "bottom-to-top" ? (
          <UpArrowIcon className="bt-icon bt-action-icon" />
        ) : (
          <DownArrowIcon className="bt-icon bt-action-icon" />
        )}
      </MenuButton>
    </>
  );
};
