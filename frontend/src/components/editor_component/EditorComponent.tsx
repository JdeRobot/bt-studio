import { useState } from "react";
import CommsManager from "../../api_helper/CommsManager";

import "./EditorComponent.css";
import {
  ResizableColumn,
  ResizableRow,
} from "./ResizableComponents";
import FileEditor from "./file_editor/FileEditor";
import Explorer, { Entry, ExplorerEntry } from "./explorer/Explorer";

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
  extra_editors,
  viewers,
  layout,
  options,
}: {
  commsManager: CommsManager | null;
  project: string;
  explorers: ExplorerEntry[];
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
            api={options}
          />
        </div>
        <div className="ide-column-container">
          <FileEditor
            currentFile={undefined}
            currentProjectname={project}
            isUnibotics={false}
            autosave={true}
            manager={commsManager}
            api={options}
          />
        </div>
      </ResizableRow>
      {/* <StatusBar
        dockerData={dockerData}
        resetManager={resetManager}
        /> */}
      <div className="bt-status-bar-container"></div>
    </div>
  );
};

export default EditorComponent;
