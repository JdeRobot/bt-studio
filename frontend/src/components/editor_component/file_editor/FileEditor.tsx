import React, { useEffect, useRef, useState } from "react";
import Editor, { Monaco } from "@monaco-editor/react";
import "./FileEditor.css";

import { ReactComponent as SaveIcon } from "./img/save.svg";
import { ReactComponent as SplashIcon } from "./img/logo_jderobot_monocolor.svg";
import { ReactComponent as SplashIconUnibotics } from "./img/logo_unibotics_monocolor.svg";
import { useError } from "../../error_popup/ErrorModal";
import { OptionsContext } from "../../options/Options";
import CommsManager from "../../../api_helper/CommsManager";
import { monacoEditorSnippet } from "./extras";
import { Entry } from "../explorer/Explorer";

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

const pylint_error: string[] = ["E0401", "E1101"];
const pylint_warning: string[] = ["W0611"];
const pylint_convention: string[] = [
  "C0114",
  "C0303",
  "C0304",
  "C0305",
  "C0411",
];
const pylint_refactor: string[] = [];
const pylint_fatal: string[] = [];

const getMarkerSeverity = (type: string, monaco: Monaco) => {
  switch (type) {
    case "refactor":
    case "convention":
      return monaco.MarkerSeverity.Info;
    case "error":
      return monaco.MarkerSeverity.Error;
    case "warning":
    case "fatal":
      return monaco.MarkerSeverity.Warning;
    default:
      return monaco.MarkerSeverity.Error;
  }
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

  const editorRef = useRef<any>(null);
  const monacoRef = useRef<Monaco>(null);

  const [fileContent, setFileContent] = useState<string | null>(null);
  const [fontSize, setFontSize] = useState(14);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
  const [filenameToSave, setFilenameToSave] = useState<Entry | undefined>(
    undefined,
  );
  const [language, setLanguage] = useState("python");
  const [projectToSave, setProjectToSave] = useState(currentProjectname);

  const code_analysis = (message: any) => {
    if (!editorRef.current || !monacoRef.current) return;

    const controller = new AbortController();

    const drawMarker = async () => {
      const data = message.data;

      if (!data) return;

      const model = editorRef.current.getModel();
      const pylint_data = data.pylint_output.map((pylint: any, i: number) => {
        return {
          startLineNumber: pylint.line,
          startColumn: pylint.column,
          endLineNumber:
            pylint.endLine === null ? pylint.column : pylint.endLine,
          endColumn:
            pylint.endColumn === null
              ? model.getLineMaxColumn(pylint.line)
              : pylint.endColumn,
          message: pylint.message,
          severity: getMarkerSeverity(pylint.type, monacoRef.current),
        };
      });
      monacoRef.current.editor.setModelMarkers(model, "owner", pylint_data);
    };

    drawMarker();

    return () => controller.abort();
  };

  const code_format = (message: any) => {
    if (!editorRef.current) return;

    const data = message.data;

    if (!data) return;

    setFileContent(data.formatted_code);
  };

  const handleKeyDown = async (event: any) => {
    if (event.ctrlKey && event.key === "s") {
      event.preventDefault();
      autoSave();
    }
  };

  useEffect(() => {
    if (manager === null) {
      return;
    }

    manager.subscribe("code-format", code_format);
    manager.subscribe("code-analysis", code_analysis);

    return () => {
      manager.unsubscribe("code-format", code_format);
      manager.unsubscribe("code-analysis", code_analysis);
    };
  }, [manager]);

  useEffect(() => {
    return () => {
      editorRef.current
        .getDomNode()
        .removeEventListener("keydown", handleKeyDown);
    };
  }, []);

  const handleEditorDidMount = (monaco: Monaco) => {
    monaco.editor.defineTheme("dark-theme", {
      base: "vs-dark",
      inherit: true,
      rules: [],
      colors: {
        "editor.background": "#16161d",
      },
    });
    monaco.editor.defineTheme("light-theme", {
      base: "vs",
      inherit: true,
      rules: [],
      colors: {
        "editor.background": "#e2e2e9",
      },
    });
  };

  const handleEditorMount = async (editor: any, monaco: Monaco) => {
    monacoRef.current = monaco;
    editorRef.current = editor;

    editorRef.current.getDomNode().addEventListener("keydown", handleKeyDown);

    monacoEditorSnippet(monaco, manager);

    editorRef.current.addCommand(
      monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.KeyI,
      function () {
        if (language !== "python")
          //TODO: only format for python. We could add more later
          return;

        if (manager && fileContent) {
          manager.code_format(fileContent);
        }
      },
    );

    return () => {
      editorRef.current
        .getDomNode()
        .removeEventListener("keydown", handleKeyDown);
    };
  };

  const editorOptions = {
    //
    fontSize: fontSize,
    lineNumbers: "on",
    roundedSelection: false,
    scrollBeyondLastLine: true,
    // word warp
    wordWrap: "wordWrapColumn",
    wordWrapColumn: 80,
    wrappingIndent: "indent",
    //
    minimap: { enabled: false },
    automaticLayout: true,
    tabSize: 4,
    rulers: [80],
    suggestOnTriggerCharacters: true,
    quickSuggestions: true,
    wordBasedSuggestions: true,
    //
    hover: true,
    glyphMargin: true,
    lineNumbersMinChars: 3,
    // scroll
    smoothScrolling: true,
    scrollbar: {
      vertical: "auto",
      horizontal: "auto",
      verticalScrollbarSize: 8,
      horizontalScrollbarSize: 8,
    },
  };

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

    if (filenameToSave === undefined) {
      console.log("No file to save");
      return;
    }

    if (currentFile.access === false) {
      console.log("File is Read-Only");
      alert("File is Read-Only");
      return;
    }

    try {
      await api.file.save(currentProjectname, filenameToSave, fileContent);
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
      if (filenameToSave && autosave) {
        autoSave();
      }
      setFilenameToSave(currentFile);
    } else {
      setFileContent(null);
      setHasUnsavedChanges(false);
    }
  }, [currentFile]);

  useEffect(() => {
    setFilenameToSave(undefined);
    if (currentFile) {
      handleSaveFile();
    }
    setProjectToSave(currentProjectname);
    setFileContent(null);
  }, [currentProjectname]);

  // useEffect(() => {
  //   handleSaveFile();
  // }, [forceSaveCurrent]);

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
    setFontSize((prevFontSize) => prevFontSize + 2);
  };

  const handleZoomOut = () => {
    setFontSize((prevFontSize) => Math.max(10, prevFontSize - 2));
  };

  // Code Analysis (with pylint)
  useEffect(() => {
    if (!editorRef.current || !monacoRef.current || !fileContent || !manager)
      return;

    editorRef.current.addCommand(
      monacoRef.current.KeyMod.CtrlCmd |
        monacoRef.current.KeyMod.Shift |
        monacoRef.current.KeyCode.KeyI,
      function () {
        if (language !== "python")
          //TODO: only format for python. We could add more later
          return;

        if (manager && fileContent) {
          manager.code_format(fileContent);
        }
      },
    );

    if (language !== "python") return;

    manager.code_analysis(fileContent, [
      ...pylint_error,
      ...pylint_warning,
      ...pylint_convention,
      ...pylint_refactor,
      ...pylint_fatal,
    ]);
  }, [fileContent]);

  return (
    <div className="bt-editor-container">
      <div className="bt-editor-menu">
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
        <Editor
          className="file-editor"
          width="100%"
          height="100%"
          defaultLanguage="python"
          defaultValue=""
          language={language}
          value={fileContent}
          theme={`${settings.theme.value}-theme`}
          onChange={(newContent: any) => {
            setFileContent(newContent);
            setHasUnsavedChanges(true); // Set the unsaved changes flag
          }}
          options={editorOptions}
          beforeMount={handleEditorDidMount}
          onMount={handleEditorMount}
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
