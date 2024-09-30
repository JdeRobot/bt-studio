import React from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import DiagramVisualizer from "./DiagramVisualizer";

const DiagramVisualizerContainer = ({
  projectName,
  manager,
}: {
  projectName: string;
  manager: any;
}) => {
  const [initialJson, setInitialJson] = useState("");
  const [treeStructure, setTreeStructure] = useState("");

  const getGraph = async (project_name: any) => {
    try {
      const response = await axios.get("/tree_api/get_project_graph/", {
        params: {
          project_name: project_name,
        },
      });
      if (response.data.success) {
        setInitialJson(response.data.graph_json);
      }
    } catch (error) {
      console.error("Error fetching graph:", error);
    }
  };

  const getBTTree = async (project_name: any) => {
    try {
      const response = await axios.get("/tree_api/get_tree_structure/", {
        params: {
          project_name: project_name,
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
    getGraph(projectName);
    getBTTree(projectName);
    console.log("Getting graph!");
  }, [projectName]);

  return (
    <div id="editor-container">
      {initialJson ? (
        <DiagramVisualizer
          modelJson={initialJson}
          treeStructure={treeStructure}
          manager={manager}
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default DiagramVisualizerContainer;
