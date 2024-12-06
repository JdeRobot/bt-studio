import React from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import {
  getProjectGraph,
  getSubtree,
  saveSubtree,
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

  // HELPERS

  const fetchTree = async () => {
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

  const saveSubtreeJson = async (
    previousResultJson: string,
    previousName: string,
  ) => {
    if (view === TreeViewType.Visualizer) {
      return;
    }
    try {
      // await saveSubtree(previousResultJson, projectName, previousName, settings.btOrder.value);
      await saveSubtree(previousResultJson, projectName, previousName);
    } catch (error: unknown) {
      if (error instanceof Error) {
        console.error(error);
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
    // When no subtree is selected, set the global json
    if (!subTreeName) {
      setGlobalJson(resultJson);
    }
  }, [resultJson]);

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
      if (previousSubTreeName) {
        saveSubtreeJson(previousResultJson, previousSubTreeName); // Save the correct previous subtree
      }
    }

    // We can go back again now
    setWentBack(false);

    // Fetch the new subtree or project graph
    // getBTTree();
    fetchTree();
    console.log("Getting graph!");
  }, [projectName, subTreeName]);

  useEffect(() => {
    if (treeHierarchy.length === 0) {
      setGoBack(false);
      return;
    }

    if (goBack) {
      saveSubtreeJson(resultJson, subTreeName); // Save the current subtree
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
    if (view === TreeViewType.Visualizer) {
      getBTTree();
    }
  }, [treeHierarchy]);

  useEffect(() => {
    // fetchTree();
    if (view === TreeViewType.Visualizer) {
      setInitialJson(resultJson)
      getBTTree();
    }
    console.log("Changing view!");
  }, [view, settings.btOrder.value]);

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
