import React, { useEffect, useRef, useState } from "react";
import Editor, { Monaco } from "@monaco-editor/react";
import "./FileEditor.css";

import { OptionsContext } from "../../options/Options";
import CommsManager from "../../../api_helper/CommsManager";
import { monacoEditorSnippet } from "./extras";

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
  commsManager,
  fileContent,
  setFileContent,
  saveFile,
  language,
  contentChange,
  zoomLevel,
}: {
  commsManager: CommsManager | null;
  fileContent: string;
  setFileContent: Function;
  saveFile: Function;
  language: string;
  contentChange: Function;
  zoomLevel: number;
}) => {
  const settings = React.useContext(OptionsContext);

  const editorRef = useRef<any>(null);
  const monacoRef = useRef<Monaco>(null);

  const [fontSize, setFontSize] = useState(14);

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
      saveFile();
    }
  };

  useEffect(() => {
    if (commsManager === null) {
      return;
    }

    commsManager.subscribe("code-format", code_format);
    commsManager.subscribe("code-analysis", code_analysis);

    return () => {
      commsManager.unsubscribe("code-format", code_format);
      commsManager.unsubscribe("code-analysis", code_analysis);
    };
  }, [commsManager]);

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

    monacoEditorSnippet(monaco, commsManager);

    editorRef.current.addCommand(
      monaco.KeyMod.CtrlCmd | monaco.KeyMod.Shift | monaco.KeyCode.KeyI,
      function () {
        if (language !== "python")
          //TODO: only format for python. We could add more later
          return;

        if (commsManager && fileContent) {
          commsManager.code_format(fileContent);
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

  useEffect(() => {
    setFontSize(Math.max(10, 14 + zoomLevel * 2));
  }, [zoomLevel]);

  // Code Analysis (with pylint)
  useEffect(() => {
    if (
      !editorRef.current ||
      !monacoRef.current ||
      !fileContent ||
      !commsManager
    )
      return;

    editorRef.current.addCommand(
      monacoRef.current.KeyMod.CtrlCmd |
        monacoRef.current.KeyMod.Shift |
        monacoRef.current.KeyCode.KeyI,
      function () {
        //TODO: only format for python. We could add more later
        if (language !== "python") return;

        if (commsManager && fileContent) {
          commsManager.code_format(fileContent);
        }
      },
    );

    if (language !== "python") return;

    commsManager.code_analysis(fileContent, [
      ...pylint_error,
      ...pylint_warning,
      ...pylint_convention,
      ...pylint_refactor,
      ...pylint_fatal,
    ]);
  }, [fileContent]);

  return (
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
        contentChange(true); // Set the unsaved changes flag
      }}
      options={editorOptions}
      beforeMount={handleEditorDidMount}
      onMount={handleEditorMount}
    />
  );
};

export default FileEditor;
