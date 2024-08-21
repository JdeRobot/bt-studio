import React, { useEffect, useState } from "react";
import axios from "axios";

import "./FileExplorer.css";
import TreeNode from "./TreeNode.jsx";
import MoreActionsMenu, { ContextMenuProps } from "./MoreActionsMenu.jsx";

const FileExplorer = ({
  setCurrentFilename,
  currentFilename,
  currentProjectname,
  setSelectedEntry,
  actionNodesData,
  showAccentColor,
  diagramEditorReady,
  fileList,
  fetchFileList,
  onDelete,
  onCreateFile,
  onCreateFolder,
}) => {
  const [showMenu, setShowMenu] = useState(false);
  const [menuFile, setMenuFile] = useState(null);
  const [menuGroupFile, setMenuGroupFile] = useState("");
  const [menuPosistion, setMenuPosistion] = React.useState({ x: 0, y: 0 });

  const MenuProps = new ContextMenuProps(
    showMenu,
    setShowMenu,
    menuPosistion,
    setMenuPosistion,
    menuFile,
    setMenuFile,
    menuGroupFile,
    setMenuGroupFile,
  );

  useEffect(() => {
    fetchFileList();
    setCurrentFilename("");
    setSelectedEntry("");
  }, [currentProjectname]);

  const handleFileClick = (filename) => {
    setCurrentFilename(filename);
    setSelectedEntry(filename);
  };

  const handleFolderClick = (filename) => {
    setSelectedEntry(filename);
  };

  if (Array.isArray(fileList)) {
    return (
      <div>
        {fileList.map((file) => (
          <TreeNode
            node={file}
            depth={0}
            parentGroup=""
            currentFilename={currentFilename}
            showAccentColor={showAccentColor}
            diagramEditorReady={diagramEditorReady}
            actionNodesData={actionNodesData}
            handleFileClick={handleFileClick}
            handleFolderClick={handleFolderClick}
            menuProps={MenuProps}
          />
        ))}
        {showMenu && (
          <MoreActionsMenu
            menuProps={MenuProps}
            actionNodesData={actionNodesData}
            onDelete={onDelete}
            onCreateFile={onCreateFile}
            onCreateFolder={onCreateFolder}
          />
        )}
      </div>
    );
  } else {
    return (
      <>
        <p>Create or select a project to start</p>
      </>
    );
  }
};

export default FileExplorer;
