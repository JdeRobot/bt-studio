import React, { useEffect, useState } from "react";
import "./FileEditor.css";

import { ReactComponent as SaveIcon } from "./img/save.svg";
import { ReactComponent as SplashIcon } from "./img/logo_jderobot_monocolor.svg";
import { ReactComponent as SplashIconUnibotics } from "./img/logo_unibotics_monocolor.svg";
import { useError } from "../../error_popup/ErrorModal";
import { OptionsContext } from "../../options/Options";
import CommsManager from "../../../api_helper/CommsManager";
import { Entry } from "../explorer/Explorer";
import TextEditor from "./TextEditor";

const fileTypes = {
  json: "json",
  md: "markdown",
  py: "python",
  config: "xml",
  cfg: "xml",
  xml: "xml",
  sdf: "xml",
  urdf: "xml",
  yaml: "yaml",
  repos: "yaml",
};

const FileEditor = ({
  currentFile,
  currentProjectname,
  isUnibotics,
  autosave,
  manager,
  api,
}: {
  currentFile: Entry | undefined;
  currentProjectname: string;
  isUnibotics: boolean;
  autosave: boolean;
  manager: CommsManager | null;
  api: any;
}) => {
  const { error } = useError();
  const settings = React.useContext(OptionsContext);

  const [fileContent, setFileContent] = useState<string | null>(null);
  const [zoomLevel, changeZoomLevel] = useState(0);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
  const [fileToSave, setFileToSave] = useState<Entry | undefined>(undefined);
  const [language, setLanguage] = useState("python");
  const [projectToSave, setProjectToSave] = useState(currentProjectname);

  const initFile = async (file: Entry) => {
    try {
      const content = await api.file.get(currentProjectname, currentFile);
      setFileContent(content);
      const extension = file.name.split(".").pop();
      var fileType = "textplain";
      if (extension) {
        for (const key in fileTypes) {
          if (key === extension) {
            fileType = fileTypes[key as keyof typeof fileTypes];
            break;
          }
        }
      }
      setLanguage(fileType);
      setHasUnsavedChanges(false); // Reset the unsaved changes flag when a new file is loaded
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error fetching file content: " + e.message);
        error("Error fetching file content: " + e.message);
      }
    }
  };

  const autoSave = async () => {
    console.log("Auto saving file...");

    if (currentFile === undefined || fileContent === null) {
      console.log("No content to save");
      return;
    }

    if (fileToSave === undefined) {
      console.log("No file to save");
      return;
    }

    if (currentFile.access === false) {
      console.log("File is Read-Only");
      alert("File is Read-Only");
      return;
    }

    try {
      await api.file.save(currentProjectname, fileToSave, fileContent);
      setHasUnsavedChanges(false); // Reset the unsaved changes flag
      console.log("Auto save completed");
    } catch (e) {
      if (e instanceof Error) {
        if ((e.message = "Request failed with status code 507")) {
          error("Error saving file: " + "You're using too much AWS space!");
        } else {
          console.log("Error saving file: " + e.message);
          //error("Error saving file: " + e.message);
          error("Error saving file: " + "I'm entering through the bad one");
        }
      }
    }
  };

  useEffect(() => {
    if (currentFile) {
      initFile(currentFile);
      if (fileToSave && autosave) {
        autoSave();
      }
      setFileToSave(currentFile);
    } else {
      setFileContent(null);
      setHasUnsavedChanges(false);
    }
  }, [currentFile]);

  useEffect(() => {
    setFileToSave(undefined);
    if (currentFile) {
      handleSaveFile();
    }
    setProjectToSave(currentProjectname);
    setFileContent(null);
  }, [currentProjectname]);

  const handleSaveFile = async () => {
    if (fileContent === null) {
      console.log("No content to save");
      return;
    }

    if (currentFile === undefined) {
      console.log("No file is currently selected");
      alert("No file is currently selected.");
      return;
    }

    if (currentFile.access === false) {
      console.log("File is Read-Only");
      alert("File is Read-Only");
      return;
    }

    try {
      await api.file.save(projectToSave, currentFile, fileContent);
      setHasUnsavedChanges(false); // Reset the unsaved changes flag
    } catch (e) {
      if (e instanceof Error) {
        if ((e.message = "Request failed with status code 507")) {
          error("Error saving file: " + "You're using too much AWS space");
        } else {
          console.error("Error saving file: " + e.message);
          error("Error saving file: " + e.message);
        }
      }
    }
  };

  const handleZoomIn = () => {
    changeZoomLevel((prevZoom) => prevZoom + 1);
  };

  const handleZoomOut = () => {
    changeZoomLevel((prevZoom) => prevZoom - 1);
  };

  return (
    <div className="ide-editor-container">
      <div className="bt-editor-menu">
        <div className="bt-editor-buttons">
          {hasUnsavedChanges && <div className="bt-unsaved-dot"></div>}
          <button className="bt-save-button" onClick={handleSaveFile}>
            <SaveIcon className="bt-icon" fill={"var(--icon)"} />
          </button>
          <button className="bt-save-button" onClick={handleZoomIn}>
            +
          </button>
          <button className="bt-save-button" onClick={handleZoomOut}>
            -
          </button>
        </div>
      </div>
      {fileContent !== null ? (
        <TextEditor
          commsManager={manager}
          fileContent={fileContent}
          setFileContent={setFileContent}
          saveFile={autoSave}
          language={language}
          contentChange={setHasUnsavedChanges}
          zoomLevel={zoomLevel}
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
