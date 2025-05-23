import React, { useContext, useRef } from "react";
import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import {
  getBaseTree,
  getSubtree,
  saveSubtree,
  saveBaseTree,
  getSubtreeStructure,
  getTreeStructure,
} from "../../api_helper/TreeWrapper";
import { TreeViewType, findSubtree } from "../helper/TreeEditorHelper";
import { OptionsContext } from "../options/Options";
import { useError } from "../error_popup/ErrorModal";

const MainTreeEditorContainer = ({
  projectName,
  setProjectEdited,
  saveCurrentDiagram,
  setSaveCurrentDiagram,
  updateFileExplorer,
}: {
  projectName: string;
  setProjectEdited: React.Dispatch<React.SetStateAction<boolean>>;
  saveCurrentDiagram: boolean;
  setSaveCurrentDiagram: Function;
  updateFileExplorer: Function;
}) => {
  const settings = useContext(OptionsContext);
  const { error } = useError();

  // STATES

  const [initialJson, setInitialJson] = useState("");
  const [treeStructure, setTreeStructure] = useState("");
  const [subTreeStructure, setSubTreeStructure] = useState<number[]>([]);
  const [view, changeView] = useState<TreeViewType>(TreeViewType.Editor);
  const [resultJson, setResultJson] = useState("");
  const [subTreeName, setSubTreeName] = useState("");
  const [treeHierarchy, setTreeHierarchy] = useState<string[]>([]);
  const [goBack, setGoBack] = useState(false);
  const [wentBack, setWentBack] = useState(false);

  const currentView = useRef(TreeViewType.Editor);

  //TODO:implement this always
  // Save
  const save = async (json: string, subtree: string) => {
    // Only save if the editor is active
    if (currentView.current !== TreeViewType.Editor) {
      return;
    }

    if (!json) {
      return;
    }

    // If in subtree save subtree, else save base tree
    if (subtree) {
      await saveSubtree(json, projectName, subtree);
    } else {
      console.log("Saving base tree");
      await saveBaseTree(json, projectName);
    }
    setProjectEdited(false);
  };

  // Load
  const load = async () => {
    try {
      const graph_json = subTreeName
        ? await getSubtree(subTreeName, projectName)
        : await getBaseTree(projectName);
      setInitialJson(graph_json);
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error(e);
        error(e.message);
      }
    }
  };

  // Update
  const update = () => {
    // Only save if the editor is active
    if (currentView.current !== TreeViewType.Editor) {
      return;
    }

    setInitialJson(resultJson);
  };

  // HELPERS

  const getBTTree = async () => {
    try {
      var tree_structure = await getTreeStructure(
        projectName,
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
            projectName,
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

  const trackTreeHierarchy = (newSubTreeName: string) => {
    if (newSubTreeName && !treeHierarchy.includes(newSubTreeName)) {
      console.log("Adding to hierarchy: ", newSubTreeName);
      setTreeHierarchy((prevHierarchy) => [...prevHierarchy, newSubTreeName]);
    }
  };

  // EFFECTS

  useEffect(() => {
    if (saveCurrentDiagram) {
      save(resultJson, subTreeName);
      setSaveCurrentDiagram(false);
    }
  }, [saveCurrentDiagram]);

  useEffect(() => {
    // Reset everything
    setWentBack(false);
    setGoBack(false);
    setTreeHierarchy([]);
    setSubTreeName("");

    // Fetch the new subtree or project graph
    load();
    console.log("Getting graph!");
    changeView(TreeViewType.Editor);
  }, [projectName]);

  useEffect(() => {
    if (subTreeName && !wentBack) {
      console.log("Tracking hierarchy!");
      trackTreeHierarchy(subTreeName);

      // Check if there's at least one item in the hierarchy before attempting to save
      const currentHierarchy = treeHierarchy;
      const previousResultJson = resultJson;
      const previousSubTreeName = treeHierarchy[currentHierarchy.length - 1];

      console.log("Current hierarchy: ", currentHierarchy);
      console.log("Saving subtree: ", previousSubTreeName);
      console.log("Previous result JSON before saving: ", previousResultJson);

      save(previousResultJson, previousSubTreeName);
    }

    // We can go back again now
    setWentBack(false);

    // Fetch the new subtree or project graph
    load();
    console.log("Getting graph!");
  }, [subTreeName]);

  useEffect(() => {
    if (treeHierarchy.length === 0) {
      setGoBack(false);
      return;
    }

    if (goBack) {
      save(resultJson, subTreeName);
      setTreeHierarchy((prevHierarchy) => {
        const newHierarchy = prevHierarchy.slice(0, -1);
        console.log("SET");
        setSubTreeName(newHierarchy[newHierarchy.length - 1] || "");
        return newHierarchy;
      });
      setGoBack(false);
      setWentBack(true);
    }
  }, [goBack]);

  // Add a new useEffect to log the updated treeHierarchy
  useEffect(() => {
    console.log("Updated Subtree hierarchy: ", treeHierarchy);
    if (currentView.current === TreeViewType.Visualizer) {
      getBTTree();
    }
  }, [treeHierarchy]);

  useEffect(() => {
    if (currentView.current === TreeViewType.Visualizer) {
      getBTTree();
    }
  }, [settings.btOrder.value]);

  useEffect(() => {
    save(resultJson, subTreeName).then(() => {
      update();
      currentView.current = view;
      if (view === TreeViewType.Visualizer) {
        getBTTree();
      }
    });
  }, [view]);

  return (
    <div
      id="editor-container"
      style={{ height: "100%", display: "flex", flexDirection: "column" }}
    >
      {initialJson ? (
        <TreeEditor
          modelJson={initialJson}
          setResultJson={setResultJson}
          projectName={projectName}
          setDiagramEdited={setProjectEdited}
          hasSubtrees={true}
          treeStructure={treeStructure}
          view={view}
          changeView={changeView}
          setSubTreeName={setSubTreeName}
          subTreeName={subTreeName}
          setGoBack={setGoBack}
          subTreeStructure={subTreeStructure}
          updateFileExplorer={updateFileExplorer}
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default MainTreeEditorContainer;
