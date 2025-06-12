import { ExtraEditorProps } from "../editor_component/file_editor/FileEditor";
import { getSubtreePath } from "../../api_helper/TreeWrapper";
import { useEffect, useRef, useState } from "react";
import { DiagramEngine, DiagramModel } from "@projectstorm/react-diagrams";
import EditActionModal from "./modals/EditActionModal";
import EditTagModal from "./modals/EditTagModal";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";
import TreeEditor from "./TreeEditor";
import { subscribe, unsubscribe } from "../helper/TreeEditorHelper";

const TreeEditorContainer = ({
  commsManager,
  project,
  file,
  changeFile,
  fileContent,
  setFileContent,
  contentRef,
  saveFile,
  language,
  zoomLevel,
}: ExtraEditorProps) => {
  const [editActionModalOpen, setEditActionModalOpen] = useState(false);
  const [node, setNode] = useState<BasicNodeModel | TagNodeModel | undefined>(
    undefined,
  );
  const [editTagModalOpen, setEditTagModalOpen] = useState(false);

  // Model and Engine for models use
  const [model, setModel] = useState<DiagramModel | undefined>(undefined);
  const [engine, setEngine] = useState<DiagramEngine | undefined>(undefined);

  const showRef = useRef<boolean>(true);
  const deleteCurrentCallbackRef = useRef<(e: any) => void>(() => {});
  const editCurrentCallbackRef = useRef<(e: any) => void>(() => {});
  const homeZoomCallbackRef = useRef<(e: any) => void>(() => {});
  const addNodeCallbackRef = useRef<(e: any) => void>(() => {});

  const setResultJson = (data: string) => {
    contentRef.current = data;
  };

  useEffect(() => {
    showRef.current = true;
  }, [file]);

  useEffect(() => {
    subscribe("BTEditorDeleteCurrent", deleteCurrentCallbackRef.current);
    subscribe("BTEditorEditCurrent", editCurrentCallbackRef.current);
    subscribe("BTEditorHomeZoom", homeZoomCallbackRef.current);
    subscribe("addBTNode", addNodeCallbackRef.current);

    return () => {
      unsubscribe("BTEditorDeleteCurrent", () => {});
      unsubscribe("BTEditorEditCurrent", () => {});
      unsubscribe("BTEditorHomeZoom", () => {});
      unsubscribe("addBTNode", () => {});
    };
  }, []);

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
      <TreeEditor
        fileContent={JSON.parse(JSON.parse(JSON.stringify(fileContent)))}
        setFileContent={setResultJson}
        setModalModel={setModel}
        setModalEngine={setEngine}
        enterSubtree={enterSubtree}
        setEditActionModalOpen={setEditActionModalOpen}
        setEditTagModalOpen={setEditTagModalOpen}
        setCurrentNode={setNode}
        deleteCurrentCallbackRef={deleteCurrentCallbackRef}
        editCurrentCallbackRef={editCurrentCallbackRef}
        homeZoomCallbackRef={homeZoomCallbackRef}
        addNodeCallbackRef={addNodeCallbackRef}
        render={showRef}
      />
    </>
  );
};

export default TreeEditorContainer;
