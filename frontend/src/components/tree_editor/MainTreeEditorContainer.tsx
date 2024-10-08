import React from "react";
import axios from "axios";
import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import { getProjectGraph, getSubtree } from "../../api_helper/TreeWrapper";

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
  const [resultJson, setResultJson] = useState("");
  const [subTreeName, setSubTreeName] = useState("");
  const [treeHierarchy, setTreeHierarchy] = useState([] as string[]);
  const [goBack, setGoBack] = useState(false);

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

  const trackTreeHierarchy = (newSubTreeName: string) => {
    if (newSubTreeName && !treeHierarchy.includes(newSubTreeName)) {
      console.log("Adding to hierarchy: ", newSubTreeName);
      setTreeHierarchy((prevHierarchy) => [...prevHierarchy, newSubTreeName]);
    }
  };

  useEffect(() => {
    // Fetch graph when component mounts
    fetchTree();
    if (subTreeName) {
      trackTreeHierarchy(subTreeName);
    }
    console.log("Getting graph!");
  }, [projectName, subTreeName]);

  useEffect(() => {
    if (goBack) {
      setTreeHierarchy((prevHierarchy) => {
        const newHierarchy = prevHierarchy.slice(0, -1);
        setSubTreeName(newHierarchy[newHierarchy.length - 1] || "");
        return newHierarchy;
      });
      setGoBack(false);
    }
  }, [goBack]);

  // Add a new useEffect to log the updated treeHierarchy
  useEffect(() => {
    console.log("Updated Subtree hierarchy: ", treeHierarchy);
  }, [treeHierarchy]);

  return (
    <div id="editor-container">
      {initialJson ? (
        <TreeEditor
          modelJson={initialJson}
          setResultJson={setGlobalJson}
          projectName={projectName}
          setDiagramEdited={setProjectEdited}
          hasSubtrees={true}
          setSubTreeName={setSubTreeName}
          setGoBack={setGoBack}
        />
      ) : (
        <p>Loading...</p> // Display a loading message until the graph is fetched
      )}
    </div>
  );
};

export default MainTreeEditorContainer;
