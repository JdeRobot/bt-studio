import React, { useRef } from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import {
  getProjectGraph,
  getSubtree,
  saveSubtree,
  saveBaseTree,
} from "../../api_helper/TreeWrapper";
import { TreeViewType, findSubtree } from "../helper/TreeEditorHelper";
import { OptionsContext } from "../options/Options";

const MainTreeEditorContainer = ({
  projectName,
  setProjectEdited,
  setGlobalJson,
  modelJson,
}: {
  projectName: string;
  setProjectEdited: React.Dispatch<React.SetStateAction<boolean>>;
  setGlobalJson: Function;
  modelJson: any;
}) => {
  const settings = React.useContext(OptionsContext);

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
        : await getProjectGraph(projectName);
      setInitialJson(graph_json);
    } catch (error: unknown) {
      if (error instanceof Error) {
        console.error(error);
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

  const getSubtreeStructure = async (name: string) => {
    try {
      const response = await axios.get("/tree_api/get_subtree_structure/", {
        params: {
          project_name: projectName,
          subtree_name: name,
          bt_order: settings.btOrder.value,
        },
      });
      if (response.data.success) {
        return response.data.tree_structure;
      }
    } catch (error) {
      console.error("Error fetching graph:", error);
    }
  };

  const getBTTree = async () => {
    try {
      const response = await axios.get("/tree_api/get_tree_structure/", {
        params: {
          project_name: projectName,
          bt_order: settings.btOrder.value,
        },
      });
      if (response.data.success) {
        // Navigate until root using baseTree
        var path: number[] = [];
        var tree_structure = response.data.tree_structure;
        for (let index = 0; index < treeHierarchy.length; index++) {
          var nextSubtree = treeHierarchy[index];
          if (nextSubtree) {
            var new_path = findSubtree(tree_structure, nextSubtree);
            if (new_path) {
              path = path.concat(new_path); //TODO: check if its not new_path.concat(path)
            }
            tree_structure = await getSubtreeStructure(nextSubtree);
          }
          console.log("TreePath", path);
        }

        setTreeStructure(tree_structure);
        setSubTreeStructure(path);
      }
    } catch (error) {
      console.error("Error fetching graph:", error);
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
    // When no subtree is selected, set the global json
    if (!subTreeName) {
      setGlobalJson(resultJson);
    }
  }, [resultJson]);

  useEffect(() => {
    // We can go back again now
    setWentBack(false);

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
    <div id="editor-container">
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
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default MainTreeEditorContainer;
