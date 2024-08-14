import React, { useEffect, useState } from "react";

import { ReactComponent as ClosedArrowIcon } from "./img/arrowSide.svg";
import { ReactComponent as OpenArrowIcon } from "./img/arrowDown.svg";
import { ReactComponent as ClosedFolderIcon } from "./img/closedFolder.svg";
import { ReactComponent as OpenFolderIcon } from "./img/openFolder.svg";
import FileIcon from "./FileIcon.jsx";

function TreeNode({
  node,
  depth,
  currentFilename,
  showAccentColor,
  diagramEditorReady,
  actionNodesData,
  handleFileClick,
}) {
  const [isCollapsed, setCollapsed] = useState(false);

  const handleClick = () => {
    if (node.is_dir) {
      setCollapsed(!isCollapsed);
    } else {
      handleFileClick(node.path);
      console.log(actionNodesData[node.name.replace(".py", "")]);
      console.log(diagramEditorReady);
    }
  };

  return (
    <>
      <div
        className={`file-item-container ${currentFilename === node.path ? "file-item-selected-container" : ""}`}
        onClick={() => handleClick()}
      >
        <div className={"file-item"} style={{ paddingLeft: depth * 16 + "px" }}>
          <FileIcon
            is_dir={node.is_dir}
            is_collapsed={isCollapsed}
            name={node.name}
            group={""}
          />
          <label>{node.name}</label>
          {showAccentColor && diagramEditorReady && (
            <div
              className="accent-color"
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
            currentFilename={currentFilename}
            showAccentColor={showAccentColor}
            diagramEditorReady={diagramEditorReady}
            actionNodesData={actionNodesData}
            handleFileClick={handleFileClick}
          />
        ))}
    </>
  );
}

export default TreeNode;
