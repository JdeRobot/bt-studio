import React from "react";
import { useState } from "react";
import Menu from "@mui/material/Menu";
import MenuItem from "@mui/material/MenuItem";
import "./NodeHeader.css";

const NodeMenu = () => {
  const [anchorEl, setAnchorEl] = useState(null);
  const [menuLabel, setMenuLabel] = useState("");

  const handleClick = (event: any, label: string) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
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
      return ["Patata"]; // Use the action names fetched from the API
    } else if (menuLabel === "Port values") {
      return ["Input port value", "Output port value"];
    } else if (menuLabel === "Sub Tree") {
      return ["Sub Tree"];
    }
    return [];
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  const handleSelect = (nodeType: string) => {
    console.log("Selected: " + nodeType);
    handleClose();
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
        <button
          className="node-button"
          onClick={(e) => handleClick(e, "Sub Tree")}
        >
          Sub Tree
        </button>
      </div>

      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={handleClose}>
        {getMenuItems().map((item: string) => (
          <MenuItem key={item} onClick={() => handleSelect(item)}>
            {item}
          </MenuItem>
        ))}
      </Menu>
    </div>
  );
};

export default NodeMenu;
