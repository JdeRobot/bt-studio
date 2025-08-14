import { useContext, useRef } from "react";
import { useState, useEffect } from "react";
import {
  getBaseTree,
  getSubtree,
  getSubtreeStructure,
  getTreeStructure,
} from "../../api_helper/TreeWrapper";
import { findSubtree } from "../helper/TreeEditorHelper";
import { OptionsContext } from "../options/Options";
import { useError } from "jderobot-ide-interface";
import TreeMonitor from "./TreeMonitor";

const TreeMonitorContainer = ({
  commsManager,
  project,
}: {
  commsManager: any;
  project: string;
}) => {
  const settings = useContext(OptionsContext);
  const { error } = useError();

  const [initialJson, setInitialJson] = useState("");
  const [treeStructure, setTreeStructure] = useState("");
  const [subTreeStructure, setSubTreeStructure] = useState<number[]>([]);
  const [treeHierarchy, setTreeHierarchy] = useState<string[]>([]);
  const [goBack, setGoBack] = useState(false);
  const [wentBack, setWentBack] = useState(false);

  const showRef = useRef<boolean>(true);
  const contentRef = useRef<string>("");
  const subtreeNameRef = useRef<string>("");

  useEffect(() => {
    if (treeHierarchy.length === 0) {
      setGoBack(false);
      return;
    }

    if (goBack) {
      setTreeHierarchy((prevHierarchy) => {
        const newHierarchy = prevHierarchy.slice(0, -1);
        enterSubtree(newHierarchy[newHierarchy.length - 1]);
        return newHierarchy;
      });
      setGoBack(false);
      setWentBack(true);
    }
  }, [goBack]);

  useEffect(() => {
    // TODO: also do this if the file changes
    // Reset everything
    setWentBack(false);
    setGoBack(false);
    setTreeHierarchy([]);

    // Fetch the new subtree or project graph
    load();
  }, [, project]);

  useEffect(() => {
    // Reset everything
    setWentBack(false);
    setGoBack(false);
    setTreeHierarchy([]);

    // Fetch the new subtree or project graph
    load();
  }, []);

  const setResultJson = (data: string) => {
    contentRef.current = data;
  };

  const load = async (subtree?: string) => {
    console.log("load Subtree");
    try {
      const graph_json = subtree
        ? await getSubtree(subtree, project)
        : await getBaseTree(project);
      setInitialJson(graph_json);
      showRef.current = true;
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error(e);
        error(e.message);
      }
    }
  };

  const getBTTree = async () => {
    try {
      var tree_structure = await getTreeStructure(
        project,
        settings.btOrder.value,
      );
      // Navigate until root using baseTree
      var path: number[] = [];
      for (let index = 0; index < treeHierarchy.length; index++) {
        var nextSubtree = treeHierarchy[index];
        if (nextSubtree) {
          var new_path = findSubtree(tree_structure, nextSubtree);
          if (new_path) {
            path = path.concat(new_path); //TODO: check if its not new_path.concat(path)
          }
          tree_structure = await getSubtreeStructure(
            project,
            nextSubtree,
            settings.btOrder.value,
          );
        }
        console.log("TreePath", path);
      }

      setTreeStructure(tree_structure);
      setSubTreeStructure(path);
    } catch (e: unknown) {
      console.error("Error fetching graph:", e);
      if (e instanceof Error) {
        error(e.message);
      }
    }
  };

  const enterSubtree = async (name?: string) => {
    if (name && !wentBack && !treeHierarchy.includes(name)) {
      setTreeHierarchy((prevHierarchy) => [...prevHierarchy, name]);
    }

    subtreeNameRef.current = name || "";
    setWentBack(false);
    load(name);
  };

  useEffect(() => {
    getBTTree();
  }, [settings.btOrder.value, treeHierarchy]);

  return (
    <TreeMonitor
      modelJson={initialJson}
      setResultJson={setResultJson}
      manager={commsManager}
      treeStructure={treeStructure}
      setGoBack={setGoBack}
      subTreeName={subtreeNameRef}
      subTreeStructure={subTreeStructure}
      enterSubtree={enterSubtree}
      render={showRef}
    />
  );
};

export default TreeMonitorContainer;
