import React from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import { getProjectGraph } from "../../api_helper/TreeWrapper";
import { TreeViewType } from "../helper/TreeEditorHelper";

const MainTreeEditorContainer = ({
  projectName,
  setProjectEdited,
  setGlobalJson,
  modelJson
}: {
  projectName: string;
  setProjectEdited: React.Dispatch<React.SetStateAction<boolean>>;
  setGlobalJson: Function;
  modelJson:any;
}) => {
  const [initialJson, setInitialJson] = useState("");
  const [treeStructure, setTreeStructure] = useState("");
  const [view, changeView] = useState<TreeViewType>(TreeViewType.Editor);

  const fetchProjectGraph = async () => {
    try {
      const graph_json = await getProjectGraph(projectName);
      setInitialJson(graph_json);
    } catch (error: unknown) {
      if (error instanceof Error) {
        console.error(error);
      }
    }
  };

  const getBTTree = async () => {
    try {
      const response = await axios.get("/tree_api/get_tree_structure/", {
        params: {
          project_name: projectName,
        },
      });
      if (response.data.success) {
        setTreeStructure(response.data.tree_structure);
      }
    } catch (error) {
      console.error("Error fetching graph:", error);
    }
  };

  useEffect(() => {
    // Fetch graph when component mounts<
    fetchProjectGraph();
    getBTTree();
    console.log("Getting graph!");
  }, [projectName]);

  useEffect(() => {
    setInitialJson(modelJson);
    getBTTree();
    console.log("Changing view!");
  }, [view])

  return (
    <div id="editor-container">
      {initialJson ? (
        <TreeEditor
          modelJson={initialJson}
          setResultJson={setGlobalJson}
          projectName={projectName}
          setDiagramEdited={setProjectEdited}
          hasSubtrees={true}
          treeStructure={treeStructure}
          view={view}
          changeView={changeView}
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default MainTreeEditorContainer;
