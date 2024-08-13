import React, { useEffect, useState } from "react";

import { ReactComponent as ClosedArrowIcon } from "./img/arrowSide.svg";
import { ReactComponent as OpenArrowIcon } from "./img/arrowDown.svg";

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
          {node.is_dir && (
            <>
              {isCollapsed ? (
                <OpenArrowIcon className="arrow-icon" stroke={"var(--icon)"} />
              ) : (
                <ClosedArrowIcon
                  className="arrow-icon"
                  stroke={"var(--icon)"}
                />
              )}
            </>
          )}
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
