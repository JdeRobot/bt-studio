import React, { useEffect, useState } from "react";

import { ReactComponent as ActionIcon } from "./img/action.svg";
import FileIcon from "./FileIcon.jsx";

function TreeNode({
  node,
  depth,
  parentGroup,
  currentFilename,
  showAccentColor,
  diagramEditorReady,
  actionNodesData,
  handleFileClick,
  handleFolderClick,
  menuProps,
}) {
  const [isCollapsed, setCollapsed] = useState(false);
  const [group, setGroup] = useState(parentGroup);

  useEffect(() => {
    if (node.is_dir) {
      if (node.name === "actions") {
        setGroup("Action");
      }
    }
  }, []);

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
          {showAccentColor && diagramEditorReady && (
            <div
              className="bt-accent-color"
              style={{
                backgroundColor: actionNodesData[node.name.replace(".py", "")]
                  ? actionNodesData[node.name.replace(".py", "")]["color"]
                  : "none",
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
            showAccentColor={showAccentColor}
            diagramEditorReady={diagramEditorReady}
            actionNodesData={actionNodesData}
            handleFileClick={handleFileClick}
            handleFolderClick={handleFolderClick}
            menuProps={menuProps}
          />
        ))}
    </>
  );
}

export default TreeNode;
