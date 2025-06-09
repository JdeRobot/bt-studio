import CommsManager from "../../api_helper/CommsManager";
import { Entry } from "../file_browser/FileBrowser";
import { getSubtreePath } from "../../api_helper/TreeWrapper";
import { useContext, useEffect, useRef, useState } from "react";
import { OptionsContext } from "../options/Options";
import { DiagramEngine, DiagramModel } from "@projectstorm/react-diagrams";
import EditActionModal from "./modals/EditActionModal";
import EditTagModal from "./modals/EditTagModal";
import { BasicNodeModel } from "../tree_editor/nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "../tree_editor/nodes/tag_node/TagNodeModel";
import DiagramEditor from "./TreeEditor";

const TreeEditor = ({
  commsManager,
  project,
  file,
  changeFile,
  fileContent,
  setFileContent,
  extraContent,
  saveFile,
  language,
  zoomLevel,
}: {
  commsManager: CommsManager | null;
  project: string;
  file: Entry;
  changeFile: Function;
  fileContent: string;
  setFileContent: Function;
  extraContent: React.MutableRefObject<string>;
  saveFile: Function;
  language: string;
  zoomLevel: number;
}) => {
  const settings = useContext(OptionsContext);

  const [editActionModalOpen, setEditActionModalOpen] = useState(false);
  const [node, setNode] = useState<BasicNodeModel | TagNodeModel | undefined>(
    undefined,
  );
  const [editTagModalOpen, setEditTagModalOpen] = useState(false);
  const [btOrder, setBtOrder] = useState(settings.btOrder.default_value);

  // Model and Engine for models use
  const [model, setModel] = useState<DiagramModel | undefined>(undefined);
  const [engine, setEngine] = useState<DiagramEngine | undefined>(undefined);
  const showRef = useRef<boolean>(true);

  useEffect(() => {
    setBtOrder(settings.btOrder.value);
  }, [settings.btOrder.value]);

  const setResultJson = (data: string) => {
    extraContent.current = data;
  };

  useEffect(() => {
    showRef.current = true;
  }, [file]);

  const enterSubtree = async (name?: string) => {
    if (name) {
      showRef.current = false;
      changeFile({
        name: `${name}.json`,
        is_dir: false,
        path: await getSubtreePath(project, name),
        group: "Trees",
        access: true,
        files: [],
      });
    }
  };

  // useEffect(() => {
  //   // TODO: move this to App.tsx
  //   resetActionFrames();
  // }, [projectName]);

  const onEditActionModalClose = () => {
    setEditActionModalOpen(false);
    setNode(undefined);
  };

  const onEditTagModalClose = () => {
    setEditTagModalOpen(false);
    setNode(undefined);
  };

  return (
    <>
      {showRef.current && (
        <>
          {node && model && engine && (
            <>
              {node instanceof BasicNodeModel && (
                <EditActionModal
                  setFileContent={setResultJson}
                  isOpen={editActionModalOpen}
                  onClose={onEditActionModalClose}
                  currentActionNode={node}
                  model={model}
                  engine={engine}
                />
              )}
              {node instanceof TagNodeModel && (
                <EditTagModal
                  setFileContent={setResultJson}
                  isOpen={editTagModalOpen}
                  onClose={onEditTagModalClose}
                  currentActionNode={node}
                  model={model}
                  engine={engine}
                />
              )}
            </>
          )}
          <DiagramEditor
            fileContent={JSON.parse(JSON.parse(JSON.stringify(fileContent)))}
            setFileContent={setResultJson}
            hasSubtrees={true}
            setModalModel={setModel}
            setModalEngine={setEngine}
            enterSubtree={enterSubtree}
            setEditActionModalOpen={setEditActionModalOpen}
            setEditTagModalOpen={setEditTagModalOpen}
            setCurrentNode={setNode}
          />
          <button className="bt-order-indicator" title={"BT Order: " + btOrder}>
            <svg
              className="w-6 h-6 text-gray-800 dark:text-white"
              aria-hidden="true"
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              fill="none"
              viewBox="0 0 24 24"
            >
              {btOrder === "bottom-to-top" ? (
                <path
                  stroke="var(--icon)"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth="2"
                  d="M12 6v13m0-13 4 4m-4-4-4 4"
                />
              ) : (
                <path
                  stroke="var(--icon)"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth="2"
                  d="M12 19V5m0 14-4-4m4 4 4-4"
                />
              )}
            </svg>
          </button>
        </>
      )}
    </>
  );
};

export default TreeEditor;
