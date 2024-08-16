import React, { useEffect, useState } from "react";
import Menu from "@mui/material/Menu";
import MenuItem from "@mui/material/MenuItem";
import "./NodeHeader.css";
import axios from "axios";

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
}: {
  projectName: string;
  onAddNode: Function;
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
    </div>
  );
};

export default NodeMenu;
