import React, { useEffect, useState } from "react";

import "./FileExplorer.css";
import TreeNode from "./TreeNode";
import MoreActionsMenu, { ContextMenuProps } from "./MoreActionsMenu";
import { Entry } from "../FileBrowser";

const FileExplorer = ({
  setCurrentFilename,
  currentFilename,
  currentProjectname,
  setSelectedEntry,
  fileList,
  fetchFileList,
  onDelete,
  onCreateFile,
  onCreateFolder,
  onUpload,
  onDownload,
  onRename,
}: {
  setCurrentFilename: Function;
  currentFilename: string;
  currentProjectname: string;
  setSelectedEntry: Function;
  fileList: Entry[];
  fetchFileList: Function;
  onDelete: Function;
  onCreateFile: Function;
  onCreateFolder: Function;
  onUpload: Function;
  onDownload: Function;
  onRename: Function;
}) => {
  const [showMenu, setShowMenu] = useState<boolean>(false);
  const [menuFile, setMenuFile] = useState<Entry | undefined>(undefined);
  const [menuGroupFile, setMenuGroupFile] = useState<string>("");
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
    console.log("The file list is: ", fileList);
    if (Array.isArray(fileList)) {
      console.log("Yes it is an array");
    }
  }, [currentProjectname]);

  const handleFileClick = (file: Entry) => {
    setCurrentFilename(file.path);
    setSelectedEntry(file);
  };

  const handleFolderClick = (file: Entry) => {
    setSelectedEntry(file);
  };

  if (Array.isArray(fileList)) {
    return (
      <div className="bt-sidebar-entry-contents">
        {fileList.map((file) => (
          <TreeNode
            node={file}
            depth={0}
            parentGroup=""
            currentFilename={currentFilename}
            handleFileClick={handleFileClick}
            handleFolderClick={handleFolderClick}
            menuProps={MenuProps}
          />
        ))}
        {showMenu && (
          <MoreActionsMenu
            menuProps={MenuProps}
            onDelete={onDelete}
            onCreateFile={onCreateFile}
            onCreateFolder={onCreateFolder}
            onUpload={onUpload}
            onDownload={onDownload}
            onRename={onRename}
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
