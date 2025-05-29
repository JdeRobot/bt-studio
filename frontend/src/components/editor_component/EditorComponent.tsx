import { useState } from "react";
import CommsManager from "../../api_helper/CommsManager";

import "./EditorComponent.css";
import { ResizableColumn, ResizableHoriz } from "./ResizableComponents";
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
  layout: string;
  options: any;
}) => {
  const [currentFile, setCurrentFile] = useState<Entry | undefined>(undefined);

  return (
    <div className="ide-container-horiz">
      <div className="ide-container">
        {explorers.length > 0 && (
          <ResizableHoriz width={20} max={40} snap={[0]}>
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
          </ResizableHoriz>
        )}
        <ResizableHoriz width={40} max={60} snap={[0]}>
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
        </ResizableHoriz>
        <div className="ide-viewers-container">
          <FileEditor
            currentFile={undefined}
            currentProjectname={project}
            isUnibotics={false}
            autosave={true}
            manager={commsManager}
            api={options}
          />
        </div>
      </div>
      {/* <StatusBar
        showSim={showSim}
        setSimVisible={showVNCSim}
        showTerminal={showTerminal}
        setTerminalVisible={showVNCTerminal}
        dockerData={dockerData}
        resetManager={resetManager}
        /> */}
      <div className="bt-status-bar-container"></div>
    </div>
  );
};

export default EditorComponent;
