import { useState, useEffect } from "react";
import TreeEditor from "./TreeEditor";
import CommsManager from "../../api_helper/CommsManager";

type Props = {
  commsManager: CommsManager | null;
  fileContent: string;
  setFileContent: Function;
  saveFile: Function;
  language: string;
  zoomLevel: number;
};

const MainTreeEditorContainer = ({
  commsManager,
  fileContent,
  setFileContent,
  saveFile,
  language,
  zoomLevel,
}: {
  commsManager: CommsManager | null;
  fileContent: string;
  setFileContent: Function;
  saveFile: Function;
  language: string;
  zoomLevel: number;
}) => {
  const [subTreeName, setSubTreeName] = useState("");
  const [treeHierarchy, setTreeHierarchy] = useState<string[]>([]);
  const [goBack, setGoBack] = useState(false);
  const [wentBack, setWentBack] = useState(false);

  // HELPERS

  const trackTreeHierarchy = (newSubTreeName: string) => {
    if (newSubTreeName && !treeHierarchy.includes(newSubTreeName)) {
      console.log("Adding to hierarchy: ", newSubTreeName);
      setTreeHierarchy((prevHierarchy) => [...prevHierarchy, newSubTreeName]);
    }
  };

  useEffect(() => {
    // Reset everything
    setWentBack(false);
    setGoBack(false);
    setTreeHierarchy([]);
    setSubTreeName("");

    // Fetch the new subtree or project graph
    console.log("Getting graph!");
  }, []);

  useEffect(() => {
    if (subTreeName && !wentBack) {
      console.log("Tracking hierarchy!");
      trackTreeHierarchy(subTreeName);

      // Check if there's at least one item in the hierarchy before attempting to save
      const currentHierarchy = treeHierarchy;
      const previousSubTreeName = treeHierarchy[currentHierarchy.length - 1];

      console.log("Current hierarchy: ", currentHierarchy);
      console.log("Saving subtree: ", previousSubTreeName);
      saveFile();
    }

    // We can go back again now
    setWentBack(false);

    // Fetch the new subtree or project graph
    console.log("Getting graph!");
  }, [subTreeName]);

  useEffect(() => {
    if (treeHierarchy.length === 0) {
      setGoBack(false);
      return;
    }

    if (goBack) {
      saveFile();
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

  return (
    <div
      id="editor-container"
      style={{ height: "100%", display: "flex", flexDirection: "column" }}
    >
      <TreeEditor
        fileContent={fileContent}
        setFileContent={setFileContent}
        hasSubtrees={true}
        setSubTreeName={setSubTreeName}
        subTreeName={subTreeName}
        setGoBack={setGoBack}
      />
    </div>
  );
};

MainTreeEditorContainer.defaultProps = {
  commsManager: null,
  fileContent: "",
  setFileContent: () => {},
  saveFile: () => {},
  language: "",
  zoomLevel: 0,
};

export default MainTreeEditorContainer;
