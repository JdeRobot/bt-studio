import React from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import DiagramEditor from "./DiagramEditor";

const EditorContainer = ({
  projectName,
  setProjectEdited,
  setGlobalJson,
}: {
  projectName: string;
  setProjectEdited: Function;
  setGlobalJson: Function;
}) => {
  const [initialJson, setInitialJson] = useState("");

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

  useEffect(() => {
    // Fetch graph when component mounts<
    getGraph(projectName);
    console.log("Getting graph!");
  }, [projectName]);

  return (
    <div id="editor-container">
      {initialJson ? (
        <DiagramEditor
          modelJson={initialJson}
          setResultJson={setGlobalJson}
          projectName={projectName}
          setDiagramEdited={setProjectEdited}
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default EditorContainer;
