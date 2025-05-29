import { useEffect, useRef } from "react";

import "./MoreActionsMenu.css";
import { Entry } from "../Explorer";

function MoreActionsMenu({
  menuProps,
  onDelete,
  onCreateFile,
  onCreateFolder,
  onUpload,
  onDownload,
  onRename,
}: {
  menuProps: ContextMenuProps;
  onDelete: Function;
  onCreateFile: Function;
  onCreateFolder: Function;
  onUpload: Function;
  onDownload: Function;
  onRename: Function;
}) {
  const menuRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (menuProps.isShown) {
      document.addEventListener("mousedown", handleClickOutside);
    } else {
      document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [menuProps.isShown]);

  const handleClickOutside = (event: any) => {
    if (menuRef.current && !menuRef.current.contains(event.target)) {
      menuProps.showCallback(false);
    }
  };

  const closeMenu = () => {
    menuProps.showCallback(false);
  };

  return (
    <div
      className="bt-menu-backdrop"
      style={{ display: menuProps.isShown ? "inline-block" : "none" }}
      ref={menuRef}
    >
      <div
        className="bt-more-actions-menu"
        style={{ top: menuProps.position.y, left: menuProps.position.x }}
      >
        {menuProps.file?.group !== "Trees" && (
          <div
            className="bt-more-actions-menu-entry"
            onClick={() => {
              onRename(menuProps.file);
              closeMenu();
            }}
          >
            <label>Rename</label>
          </div>
        )}
        {true && (
          <div
            className="bt-more-actions-menu-entry"
            onClick={() => {
              onDownload(menuProps.file);
              closeMenu();
            }}
          >
            <label>Download</label>
          </div>
        )}
        <div
          className="bt-more-actions-menu-entry"
          onClick={() => {
            onDelete(menuProps.file!.path, menuProps.file!.is_dir);
            closeMenu();
          }}
        >
          <label>Delete</label>
        </div>
        {!menuProps.file!.is_dir &&
          menuProps.file?.group === "Action" &&
          false && ( // TODO: disabled
            <>
              <div className="bt-more-actions-menu-divider" />
              <div
                className="bt-more-actions-menu-entry"
                onClick={() => {
                  // TODO open the same menu that in the diagram
                  console.log("Edit Action");
                  closeMenu();
                }}
              >
                <label>Edit Action</label>
              </div>
            </>
          )}
        {menuProps.file?.group !== "Trees" && (
          <>
            <div className="bt-more-actions-menu-divider" />
            <div
              className="bt-more-actions-menu-entry"
              onClick={() => {
                onCreateFile(menuProps.file);
                closeMenu();
              }}
            >
              <label>New File</label>
            </div>
            <div
              className="bt-more-actions-menu-entry"
              onClick={() => {
                onCreateFolder(menuProps.file);
                closeMenu();
              }}
            >
              <label>New Folder</label>
            </div>
            <div
              className="bt-more-actions-menu-entry"
              onClick={() => {
                onUpload(menuProps.file);
                closeMenu();
              }}
            >
              <label>Upload</label>
            </div>
          </>
        )}
      </div>
    </div>
  );
}

export default MoreActionsMenu;

export class ContextMenuProps {
  public isShown: boolean;
  public showCallback: Function;
  public position: { x: number; y: number };
  public setPositionCallback: Function;
  public file: Entry | undefined;
  public setFile: Function;

  constructor(
    isShown: boolean,
    showCallback: Function,
    position: { x: number; y: number },
    setPositionCallback: Function,
    file: Entry | undefined,
    setFile: Function,
  ) {
    this.isShown = isShown;
    this.showCallback = showCallback;
    this.position = position;
    this.setPositionCallback = setPositionCallback;
    this.file = file;
    this.setFile = setFile;
  }

  showMoreActionsMenu(event: any, file: Entry) {
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
  }
}
