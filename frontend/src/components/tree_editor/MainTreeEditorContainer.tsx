import React from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import { getProjectGraph } from "../../api_helper/TreeWrapper";

const MainTreeEditorContainer = ({
  projectName,
  setProjectEdited,
  setGlobalJson,
}: {
  projectName: string;
  setProjectEdited: React.Dispatch<React.SetStateAction<boolean>>;
  setGlobalJson: Function;
}) => {
  const [initialJson, setInitialJson] = useState("");

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

  useEffect(() => {
    // Fetch graph when component mounts<
    fetchProjectGraph();
    console.log("Getting graph!");
  }, [projectName]);

  return (
    <div id="editor-container">
      {initialJson ? (
        <TreeEditor
          modelJson={initialJson}
          setResultJson={setGlobalJson}
          projectName={projectName}
          setDiagramEdited={setProjectEdited}
          hasSubtrees={true}
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default MainTreeEditorContainer;
