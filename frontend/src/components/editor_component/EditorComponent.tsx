import { useState } from "react";
import CommsManager from "../../api_helper/CommsManager";

import "./EditorComponent.css";
import {
  CollapsableResizableColumn,
  ResizableColumn,
  ResizableRow,
} from "./ResizableComponents";
import FileEditor, { EditorsEntry } from "./file_editor/FileEditor";
import Explorer, { Entry, ExplorerEntry } from "./explorer/Explorer";
import StatusBar from "./status_bar/StatusBar";

interface ViewersEntry {
  component: JSX.Element;
  icon: JSX.Element;
  name: string;
  active: boolean;
  activate: React.Dispatch<React.SetStateAction<boolean>>;
}

const EditorComponent = ({
  commsManager,
  resetManager,
  project,
  explorers,
  editorApi,
  extraEditors,
  viewers,
  layout,
  options,
}: {
  commsManager: CommsManager | null;
  resetManager: Function;
  project: string;
  explorers: ExplorerEntry[];
  editorApi: any;
  extraEditors: EditorsEntry[];
  viewers: ViewersEntry[];
  layout: "only-editor" | "only-viewers" | "both";
  options: any;
}) => {
  const [currentFile, setCurrentFile] = useState<Entry | undefined>(undefined);

  return (
    <div className="ide-container-horiz">
      <ResizableRow
        baseWidth={[20, 40, 40]}
        maxWidth={[40, 60, 60]}
        showExplorer={explorers.length > 0}
        layout={layout}
      >
        {explorers.length > 0 && (
          <ResizableColumn>
            {explorers.map((explorer) => (
              <Explorer
                setCurrentFile={setCurrentFile}
                currentFile={currentFile}
                project={project}
                api={explorer}
              />
            ))}
          </ResizableColumn>
        )}
        <div className="ide-column-container">
          <FileEditor
            currentFile={currentFile}
            changeCurrentFile={setCurrentFile}
            currentProjectname={project}
            isUnibotics={false}
            autosave={true}
            manager={commsManager}
            api={editorApi}
            extraEditors={extraEditors}
          />
        </div>
        <div className="ide-column-container">
          <ViewersContainer viewers={viewers} options={options} />
        </div>
      </ResizableRow>
      <StatusBar
        project={project}
        commsManager={commsManager}
        resetManager={resetManager}
      />
    </div>
  );
};

export default EditorComponent;

const ViewersContainer = ({
  viewers,
  options,
}: {
  viewers: ViewersEntry[];
  options: any;
}) => {
  const [visibility, setVisibility] = useState<boolean[]>(
    Array(viewers.length).fill(false),
  );

  const toggleVisibility = (index: number) => {
    setVisibility(
      visibility.map((state, i) => {
        if (index === i) {
          return !state;
        } else {
          return state;
        }
      }),
    );
  };

  return (
    <div className="ide-editor-container">
      <div className="ide-viewer-menu">
        <div className="ide-viewer-buttons">
          {viewers.map((viewer, index) => (
            <button
              className={`ide-viewer-toggle-button ${visibility[index] ? "active" : ""}`}
              onClick={() => toggleVisibility(index)}
            >
              {viewer.icon}
            </button>
          ))}
        </div>
      </div>
      <CollapsableResizableColumn state={visibility}>
        {viewers.map((viewer) => viewer.component)}
      </CollapsableResizableColumn>
    </div>
  );
};
