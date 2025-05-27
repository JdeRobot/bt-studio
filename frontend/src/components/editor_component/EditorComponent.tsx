import { useState } from "react";
import CommsManager from "../../api_helper/CommsManager";

import "./EditorComponent.css";
import { ResizableColumn, ResizableHoriz } from "./ResizableComponents";
import FileEditor from "../file_editor/FileEditor";

interface ViewersEntry {
  component: JSX.Element;
  icon: JSX.Element;
  name: string;
  active: boolean;
  activate: React.Dispatch<React.SetStateAction<boolean>>;
}

const EditorComponent = ({
  commsManager,
  explorers,
  extra_editors,
  viewers,
  options,
}: {
  commsManager: CommsManager | null;
  explorers: any[];
  extra_editors: any[];
  viewers: ViewersEntry[];
  options: any;
}) => {
  const [currentFile, setCurrentFile] = useState<string | undefined>(undefined);
  const [autosaveEnabled, setAutosave] = useState<boolean>(true);
  const [forceSaveCurrent, setForcedSaveCurrent] = useState<boolean>(false);

  return (
    <div className="ide-container-horiz">
      <div className="ide-container">
        {explorers.length > 0 && (
          <ResizableHoriz width={20} max={40} snap={[0]}>
            <ResizableColumn>{explorers}</ResizableColumn>
          </ResizableHoriz>
        )}
        <ResizableHoriz width={40} max={60} snap={[0]}>
          <div className="ide-column-container">
            <FileEditor
              currentFilename={"actions/Turn.py"}
              currentProjectname={"laser_bump_and_go"}
              setProjectChanges={() => {}}
              isUnibotics={false}
              autosaveEnabled={autosaveEnabled}
              setAutosave={setAutosave}
              forceSaveCurrent={forceSaveCurrent}
              manager={commsManager}
            />
          </div>
        </ResizableHoriz>
        <div className="ide-viewers-container">
          <FileEditor
            currentFilename={"actions/Turn.py"}
            currentProjectname={"laser_bump_and_go"}
            setProjectChanges={() => {}}
            isUnibotics={false}
            autosaveEnabled={autosaveEnabled}
            setAutosave={setAutosave}
            forceSaveCurrent={forceSaveCurrent}
            manager={commsManager}
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
