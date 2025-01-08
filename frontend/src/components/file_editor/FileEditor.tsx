import React, { useEffect, useState } from "react";
import AceEditor from "react-ace";
import axios from "axios";
import "ace-builds/src-noconflict/mode-python";
import "ace-builds/src-noconflict/theme-monokai";
import "./FileEditor.css";

import { ReactComponent as SaveIcon } from "./img/save.svg";
import { ReactComponent as SplashIcon } from "./img/logo_jderobot_monocolor.svg";
import { ReactComponent as SplashIconUnibotics } from "./img/logo_unibotics_monocolor.svg";
import { saveFile } from "../../api_helper/TreeWrapper";

const FileEditor = ({
  currentFilename,
  currentProjectname,
  setProjectChanges,
  isUnibotics,
  autosaveEnabled,
  setAutosave,
  forceSaveCurrent,
}: {
  currentFilename: string;
  currentProjectname: string;
  setProjectChanges: Function;
  isUnibotics: boolean;
  autosaveEnabled: boolean;
  setAutosave: Function;
  forceSaveCurrent: boolean;
}) => {
  const [fileContent, setFileContent] = useState(null);
  const [fontSize, setFontSize] = useState(14);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
  const [filenameToSave, setFilenameToSave] = useState("");
  const [projectToSave, setProjectToSave] = useState(currentProjectname);

  const initFile = async () => {
    try {
      const response = await axios.get(
        `/bt_studio/get_file?project_name=${currentProjectname}&filename=${currentFilename}`,
      );
      const content = response.data.content;
      setFileContent(content);
      setHasUnsavedChanges(false); // Reset the unsaved changes flag when a new file is loaded
    } catch (error) {
      console.error("Error fetching file content:", error);
    }
  };

  const autoSave = async () => {
    console.log("Auto saving file...");

    if (fileContent == null) {
      console.log("No content to save");
      return;
    }

    try {
      await saveFile(currentProjectname, filenameToSave, fileContent);
      setHasUnsavedChanges(false); // Reset the unsaved changes flag
      setProjectChanges(false);
      console.log("Auto save completed");
    } catch (error) {
      console.error("Error saving file:", error);
    }
  };

  useEffect(() => {
    if (currentFilename != "") {
      initFile();
      if (filenameToSave && autosaveEnabled) {
        autoSave();
      }
      setAutosave(true);
      setFilenameToSave(currentFilename);
    } else {
      setFileContent(null);
      setHasUnsavedChanges(false);
    }
  }, [currentFilename]);

  useEffect(() => {
    setFilenameToSave("");
    if (currentFilename) {
      handleSaveFile();
    }
    setProjectToSave(currentProjectname);
    setFileContent(null);
  }, [currentProjectname]);

  useEffect(() => {
    handleSaveFile();
  }, [forceSaveCurrent]);

  const handleSaveFile = async () => {
    if (fileContent === null) {
      console.log("No content to save");
      return;
    }

    if (currentFilename === "") {
      console.log("No file is currently selected");
      alert("No file is currently selected.");
      return;
    }

    try {
      await saveFile(projectToSave, currentFilename, fileContent);
      setHasUnsavedChanges(false); // Reset the unsaved changes flag
      setProjectChanges(false);
    } catch (error) {
      console.error("Error saving file:", error);
    }
  };

  const handleZoomIn = () => {
    setFontSize((prevFontSize) => prevFontSize + 2);
  };

  const handleZoomOut = () => {
    setFontSize((prevFontSize) => Math.max(10, prevFontSize - 2));
  };

  return (
    <div className="bt-editor-container">
      <div className="bt-editor-menu">
        <h2>File Editor</h2>
        <div className="bt-editor-buttons">
          {hasUnsavedChanges && <div className="bt-unsaved-dot"></div>}
          <button className="bt-save-button" onClick={handleSaveFile}>
            <SaveIcon className="bt-icon" fill={"var(--icon)"} />
          </button>
        </div>
      </div>
      {fileContent !== null && (
        <div className="bt-zoom-buttons">
          <button className="bt-zoom-in" onClick={handleZoomIn}>
            +
          </button>
          <button className="bt-zoom-in" onClick={handleZoomOut}>
            -
          </button>
        </div>
      )}
      {fileContent !== null ? (
        <AceEditor
          mode="python"
          theme="monokai"
          name="fileEditor"
          width="100%"
          height="calc(100% - 50px)"
          value={fileContent}
          fontSize={fontSize}
          onChange={(newContent: any) => {
            setProjectChanges(true);
            setFileContent(newContent);
            setHasUnsavedChanges(true); // Set the unsaved changes flag
          }}
          setOptions={{
            scrollPastEnd: true,
          }}
        />
      ) : (
        <>
          {isUnibotics ? (
            <SplashIconUnibotics
              className="bt-splash-icon"
              fill="var(--header)"
            />
          ) : (
            <SplashIcon className="bt-splash-icon" fill="var(--header)" />
          )}
        </>
      )}
    </div>
  );
};

export default FileEditor;
