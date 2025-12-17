import React from "react";
import { ExtraEditorProps } from "jderobot-ide-interface";
import { getSubtreePath, getTreeData } from "BtApi/TreeWrapper";
import { useEffect, useRef, useState } from "react";
import { DiagramEngine, DiagramModel } from "@projectstorm/react-diagrams";
import EditActionModal from "./modals/EditActionModal";
import EditTagModal from "./modals/EditTagModal";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";
import TreeEditor from "./TreeEditor";
import { addActionFrameRaw, subscribe, unsubscribe } from "../helper/TreeEditorHelper";

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
  const closeEditActionModalRef = useRef<() => void>(() => {});
  const closeEditTagModalRef = useRef<() => void>(() => {});

  const setResultJson = (data: string) => {
    contentRef.current = data;
  };

  const loadData = async () => {
    const data = await getTreeData(project);
    data.forEach(element => {
      addActionFrameRaw(element.name, element.color, element.in, element.out)
    });
  }

  useEffect(() => {
    loadData()
  }, []);

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

  const onEditActionModalClose = () => {
    setEditActionModalOpen(false);
    setNode(undefined);
    closeEditActionModalRef.current();
  };

  const onEditTagModalClose = () => {
    setEditTagModalOpen(false);
    setNode(undefined);
    closeEditTagModalRef.current();
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
        closeEditActionModalRef={closeEditActionModalRef}
        closeEditTagModalRef={closeEditTagModalRef}
        render={showRef}
      />
    </>
  );
};

export default TreeEditorContainer;
