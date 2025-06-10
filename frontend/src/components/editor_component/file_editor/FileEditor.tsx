import React, { useEffect, useRef, useState } from "react";
import "./FileEditor.css";

import { ReactComponent as SaveIcon } from "./img/save.svg";
import { ReactComponent as SplashIcon } from "./img/logo_jderobot_monocolor.svg";
import { ReactComponent as SplashIconUnibotics } from "./img/logo_unibotics_monocolor.svg";
import { useError } from "../../error_popup/ErrorModal";
import { OptionsContext } from "../../options/Options";
import CommsManager from "../../../api_helper/CommsManager";
import { Entry } from "../explorer/Explorer";
import TextEditor from "./TextEditor";

export interface ExtraEditorProps {
  commsManager: CommsManager | null;
  project: string;
  file: Entry;
  changeFile: Function;
  fileContent: string;
  setFileContent: Function;
  contentRef: React.MutableRefObject<string>;
  saveFile: Function;
  language: string;
  zoomLevel: number;
}

export interface EditorsEntry {
  component: any;
  buttons: any[];
  name: string;
  language: string;
  trigger: { group: string; extension: string }[];
}

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
  changeCurrentFile,
  currentProjectname,
  isUnibotics,
  autosave,
  manager,
  api,
  extraEditors,
}: {
  currentFile: Entry | undefined;
  changeCurrentFile: Function;
  currentProjectname: string;
  isUnibotics: boolean;
  autosave: boolean;
  manager: CommsManager | null;
  api: any;
  extraEditors: EditorsEntry[];
}) => {
  const { error, warning } = useError();
  const settings = React.useContext(OptionsContext);

  const [fileContent, setFileContent] = useState<string | undefined>(undefined);
  const [zoomLevel, changeZoomLevel] = useState(0);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
  const [fileToSave, setFileToSave] = useState<Entry | undefined>(undefined);
  const [language, setLanguage] = useState("python");
  const [projectToSave, setProjectToSave] = useState(currentProjectname);
  const contentRef = useRef<string>(""); // In case some editors cannot update states

  const initFile = async (file: Entry) => {
    try {
      console.log("Loading new file...");
      const content = await api.file.get(currentProjectname, currentFile);
      const extension = file.name.split(".").pop();
      setFileContent(content);
      var fileType = "textplain";
      if (extension) {
        for (const key in fileTypes) {
          if (key === extension) {
            fileType = fileTypes[key as keyof typeof fileTypes];
            break;
          }
        }
      }

      setHasUnsavedChanges(false); // Reset the unsaved changes flag when a new file is loaded

      for (const editor of extraEditors) {
        for (const entry of editor.trigger) {
          if (
            entry.group === currentFile?.group &&
            entry.extension === currentFile?.name.split(".").pop()
          ) {
            console.log("Loading new file ended");
            return setLanguage(editor.language);
          }
        }
      }

      setLanguage(fileType);
      console.log("Loading new file ended");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error fetching file content: " + e.message);
        error("Error fetching file content: " + e.message);
      }
    }
  };

  const autoSave = async () => {
    console.log("Auto saving file...");

    if (fileContent === null) {
      console.log("No content to save");
      return;
    }

    if (fileToSave === undefined) {
      console.log("No file to save");
      return;
    }

    if (fileToSave.access === false) {
      console.log("File is Read-Only");
      warning("File is Read-Only");
      return;
    }

    var content = fileContent;

    if (contentRef.current !== "") {
      content = contentRef.current;
    }

    try {
      await api.file.save(currentProjectname, fileToSave, content);
      console.log("Auto save completed");
    } catch (e) {
      if (e instanceof Error) {
        if ((e.message = "Request failed with status code 507")) {
          error("Error saving file: " + "You're using too much AWS space!");
        } else {
          console.log("Error saving file: " + e.message);
          error("Error saving file: " + "I'm entering through the bad one");
        }
      }
    }
  };

  useEffect(() => {
    setHasUnsavedChanges(fileContent !== undefined);
  }, [fileContent]);

  useEffect(() => {
    const func = async () => {
      if (currentFile) {
        if (fileToSave && autosave) {
          await autoSave();
        }
        contentRef.current = "";
        setFileContent(undefined);
        await initFile(currentFile);
        setFileToSave(currentFile);
      } else {
        setFileContent(undefined);
        contentRef.current = "";
        setHasUnsavedChanges(false);
      }
    };
    func();
  }, [currentFile]);

  useEffect(() => {
    setFileToSave(undefined);
    if (currentFile) {
      handleSaveFile();
    }
    setProjectToSave(currentProjectname);
    setFileContent(undefined);
    contentRef.current = "";
  }, [currentProjectname]);

  const handleSaveFile = async () => {
    if (fileContent === null) {
      console.log("No content to save");
      return;
    }

    if (currentFile === undefined) {
      console.log("No file is currently selected");
      warning("No file is currently selected.");
      return;
    }

    if (currentFile.access === false) {
      console.log("File is Read-Only");
      warning("File is Read-Only");
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
          {(() => {
            for (const editor of extraEditors) {
              if (editor.language === language) {
                var list: any[] = [];
                for (const b of editor.buttons) {
                  list.push(
                    <div className="bt-editor-buttons-container">{b}</div>,
                  );
                }
                return <>{list}</>;
              }
            }
            return <></>;
          })()}
        </div>
      </div>
      {fileContent ? (
        <>
          {(() => {
            for (const editor of extraEditors) {
              if (editor.language === language) {
                return (
                  <editor.component
                    commsManager={manager}
                    project={currentProjectname}
                    file={currentFile}
                    changeFile={changeCurrentFile}
                    fileContent={fileContent}
                    setFileContent={setFileContent}
                    contentRef={contentRef}
                    saveFile={autoSave}
                    language={language}
                    zoomLevel={zoomLevel}
                  />
                );
              }
            }
            return (
              <TextEditor
                commsManager={manager}
                fileContent={fileContent}
                setFileContent={setFileContent}
                saveFile={autoSave}
                language={language}
                zoomLevel={zoomLevel}
              />
            );
          })()}
        </>
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
