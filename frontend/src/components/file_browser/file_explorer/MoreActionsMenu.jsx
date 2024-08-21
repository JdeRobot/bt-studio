import React, { useEffect, useState, useRef } from "react";

import "./MoreActionsMenu.css";

function MoreActionsMenu({ menuProps, actionNodesData, onDelete }) {
  const menuRef = useRef(null);

  useEffect(() => {
    if (menuProps.isShown) {
      document.addEventListener("mousedown", handleClickOutside);
    } else {
      document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [menuProps.isShown]);

  const handleClickOutside = (event) => {
    if (menuRef.current && !menuRef.current.contains(event.target)) {
      menuProps.showCallback(false);
    }
  };

  const closeMenu = () => {
    menuProps.showCallback(false);
  };

  return (
    <div
      className="menu-backdrop"
      style={{ display: menuProps.isShown ? "inline-block" : "none" }}
      ref={menuRef}
    >
      <div
        className="more-actions-menu"
        style={{ top: menuProps.position.y, left: menuProps.position.x }}
      >
        <div className="more-actions-menu-entry">
          <label>Rename</label>
        </div>
        {!menuProps.file.is_dir && (
          <div className="more-actions-menu-entry">
            <label>Download</label>
          </div>
        )}
        <div
          className="more-actions-menu-entry"
          onClick={() => {
            onDelete(menuProps.file.path);
            closeMenu();
          }}
        >
          <label>Delete</label>
        </div>
        {!menuProps.file.is_dir && menuProps.fileGroup === "Action" && (
          <>
            <div className="more-actions-menu-divider" />
            <div className="more-actions-menu-entry">
              <label>Edit Action</label>
            </div>
          </>
        )}
        <div className="more-actions-menu-divider" />
        <div className="more-actions-menu-entry">
          <label>New File</label>
        </div>
        <div className="more-actions-menu-entry">
          <label>New Action</label>
        </div>
        <div className="more-actions-menu-entry">
          <label>Upload</label>
        </div>
      </div>
    </div>
  );
}

export default MoreActionsMenu;

export class ContextMenuProps {
  constructor(
    isShown,
    showCallback,
    position,
    setPositionCallback,
    file,
    setFile,
    fileGroup,
    setFileGroup,
  ) {
    this.isShown = isShown;
    this.showCallback = showCallback;
    this.position = position;
    this.setPositionCallback = setPositionCallback;
    this.file = file;
    this.setFile = setFile;
    this.fileGroup = fileGroup;
    this.setFileGroup = setFileGroup;
  }

  showMoreActionsMenu(event, file, fileGroup) {
    event.preventDefault();
    event.stopPropagation();
    this.showCallback(false);
    const positionChange = {
      x: 200 + 20, // The width is set to 200
      y: event.pageY,
    };
    this.setPositionCallback(positionChange);
    this.showCallback(true);
    this.setFile(file);
    this.setFileGroup(fileGroup);
  }
}
