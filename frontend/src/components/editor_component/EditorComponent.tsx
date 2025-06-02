import { useState } from "react";
import CommsManager from "../../api_helper/CommsManager";

import "./EditorComponent.css";
import { ResizableColumn, ResizableRow } from "./ResizableComponents";
import FileEditor from "./file_editor/FileEditor";
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
  project,
  explorers,
  editorApi,
  extra_editors,
  viewers,
  layout,
  options,
}: {
  commsManager: CommsManager | null;
  project: string;
  explorers: ExplorerEntry[];
  editorApi: any;
  extra_editors: any[];
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
            currentProjectname={project}
            isUnibotics={false}
            autosave={true}
            manager={commsManager}
            api={editorApi}
          />
        </div>
        <div className="ide-column-container"></div>
      </ResizableRow>
      <StatusBar commsManager={commsManager} resetManager={() => {}} />
    </div>
  );
};

export default EditorComponent;
