import React, { useEffect, useState } from "react";

import { ReactComponent as ActionIcon } from "./img/action.svg";
import FileIcon from "./FileIcon";
import { OptionsContext } from "../../options/Options";
import { ContextMenuProps } from "./MoreActionsMenu";
import { ActionFrame, getActionFrame, subscribe, unsubscribe } from "../../helper/TreeEditorHelper";

interface Entry {
  name: string;
  is_dir: boolean;
  path: string;
  files: Entry[];
}

function TreeNode({
  node,
  depth,
  parentGroup,
  currentFilename,
  handleFileClick,
  handleFolderClick,
  menuProps,
}: {
  node: Entry;
  depth: number;
  parentGroup: string;
  currentFilename: string;
  handleFileClick: Function;
  handleFolderClick: Function;
  menuProps: ContextMenuProps;
}) {
  const [isCollapsed, setCollapsed] = useState<boolean>(false);
  const [group, setGroup] = useState<string>(parentGroup);
  const [update, setUpdate] = useState<boolean>(false);
  const [actionFrame, setActionFrame] = useState<ActionFrame | undefined>(getActionFrame(node.name.replace(".py", "")));
  const settings = React.useContext(OptionsContext);

  const callback = () => {
    setActionFrame(getActionFrame(node.name.replace(".py", "")))
    setUpdate(true)
  }

  useEffect(() => {
    if (node.is_dir) {
      if (node.name === "actions") {
        setGroup("Action");
      }
    }
    subscribe("updateAccentColor", callback);

    return () => {
      unsubscribe("updateAccentColor", () => {});
    }
  }, []);

  useEffect(() => {
    setActionFrame(getActionFrame(node.name.replace(".py", "")))
  }, [getActionFrame(node.name.replace(".py", ""))]);

  useEffect(() => {
    if (update) {
      setUpdate(false)
    }
  }, [update]);

  const handleClick = () => {
    if (node.is_dir) {
      setCollapsed(!isCollapsed);
      handleFolderClick(node);
    } else {
      handleFileClick(node);
    }
  };

  return (
    <>
      <div
        className={`bt-file-item-container ${currentFilename === node.path ? "bt-file-item-selected-container" : ""}`}
        onClick={() => handleClick()}
      >
        <div
          className={"bt-file-item"}
          style={{ paddingLeft: depth * 16 + "px" }}
        >
          <FileIcon
            is_dir={node.is_dir}
            is_collapsed={isCollapsed}
            name={node.name}
            group={parentGroup === "" ? group : parentGroup}
          />
          <label>{node.name}</label>
          {/* Add menu button */}
          <ActionIcon
            className="bt-more-action-icon bt-arrow-icon"
            stroke={"var(--icon)"}
            title={"More"}
            onClick={(e) => {
              menuProps.showMoreActionsMenu(
                e,
                node,
                parentGroup === "" ? group : parentGroup,
              );
            }}
          />
          {settings.editorShowAccentColors.value && (
            <div
              className="bt-accent-color"
              style={{
                backgroundColor: actionFrame ? actionFrame.getColor() : "none",
              }}
            />
          )}
        </div>
      </div>
      {!isCollapsed &&
        node.files.map((x) => (
          <TreeNode
            node={x}
            depth={depth + 1}
            parentGroup={group}
            currentFilename={currentFilename}
            handleFileClick={handleFileClick}
            handleFolderClick={handleFolderClick}
            menuProps={menuProps}
          />
        ))}
    </>
  );
}

export default TreeNode;
