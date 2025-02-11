import { Monaco } from "@monaco-editor/react";
import { my_snippets, Snippet } from "./snippets";
import CommsManager from "../../api_helper/CommsManager";

interface Position {
  lineNumber: number;
  column: number;
}

interface Range {
  startLineNumber: number;
  endLineNumber: number;
  startColumn: number;
  endColumn: number;
}

interface CompletionItem {
  detail?: string;
  documentation: string;
  insertText: string;
  insertTextRules: number;
  kind: number;
  label: string;
  range: Range;
}

// Main Editor Snippets
export const monacoEditorSnippet = (monaco: Monaco, manager: CommsManager| null) => {
  monaco.languages.register({ id: "python" });

  const EventEmitter = require("events");

  const bus = new EventEmitter();
  let lock = true;

  // Register a completion item provider for the new language
  monaco.languages.registerCompletionItemProvider("python", {
    triggerCharacters: [".", "("],
    provideCompletionItems: async (model: any, position: Position) => {
      lock = true;

      var word = model.getWordUntilPosition(position);
      var prevWord = model.getWordUntilPosition({
        lineNumber: position.lineNumber,
        column: position.column - 1,
      });

      var range: Range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };

      // Add basic snippets only if not prevWord
      if (prevWord.word === "") {
        var snippets: CompletionItem[] = snippetsBuilderV2(monaco, range);
      } else {
        var snippets: CompletionItem[] = [];
      }

      // Check if the Robotics Backend is connected
      // Call the RAM for autocompletion
      if (manager === null) {
        return snippets;
      } 

      try {
        manager.code_autocomplete(
          model.getValue(),
          position.lineNumber,
          word.endColumn - 1
        );
      } catch (error) {
        return snippets;
      }

      const callback = (message: any) => {
        const data = message.data;

        if (!data) return;

        const new_completions = data.completions;

        new_completions.forEach((snippet: Snippet) => {
          snippets.push({
            label: snippet.label,
            kind: snippetKind(snippet.type, monaco),
            detail: snippet.detail,
            documentation: snippet.docstring,
            insertText: snippet.code,
            insertTextRules:
              monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
            range: range,
          });
        });

        lock = false;
        bus.emit("unlocked");
      };

      manager.subscribeOnce("code-autocomplete", callback);

      if (lock) await new Promise((resolve) => bus.once("unlocked", resolve));

      return { suggestions: snippets };
    },
  });
};

// Snippets Builder
export const snippetKind = (kind: string, monaco: Monaco) => {
  switch (kind) {
    case "variable":
      return monaco.languages.CompletionItemKind.Variable;
    case "class":
      return monaco.languages.CompletionItemKind.Class;
    case "param":
      return monaco.languages.CompletionItemKind.TypeParameter;
    case "path":
      return monaco.languages.CompletionItemKind.File;
    case "property":
      return monaco.languages.CompletionItemKind.Property;
    case "statement":
      return monaco.languages.CompletionItemKind.Function;
    case "instance":
      return monaco.languages.CompletionItemKind.Class;
    case "module":
      return monaco.languages.CompletionItemKind.Module;
    case "method":
      return monaco.languages.CompletionItemKind.Method;
    case "snippet":
      return monaco.languages.CompletionItemKind.Snippet;
    case "keyword":
      return monaco.languages.CompletionItemKind.Keyword;
    case "function":
      return monaco.languages.CompletionItemKind.Function;
    default:
      return monaco.languages.CompletionItemKind.Variable;
  }
};

export const snippetsBuilderV2 = (monaco: Monaco, range: Range) => {
  const snippets: CompletionItem[] = [];
  let importSnippets: Snippet[] = my_snippets;

  if (!importSnippets || !importSnippets.length) return [];

  importSnippets.forEach((snippet: Snippet) => {
    if (snippet.label && snippet.code) {
      snippets.push({
        label: snippet.label,
        kind: snippetKind(snippet.type, monaco),
        detail: snippet.detail,
        documentation: snippet.docstring,
        insertText: snippet.code,
        insertTextRules:
          monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
        range: range,
      });
    }
  });

  return snippets;
};
