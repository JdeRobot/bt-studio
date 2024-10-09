import React, { useState, useEffect } from "react";
import Modal from "@mui/material/Modal";
import Box from "@mui/material/Box";
import IconButton from "@mui/material/IconButton";
import Typography from "@mui/material/Typography";
import CloseIcon from "@mui/icons-material/Close";
import TreeEditor from "../TreeEditor";
import { getSubtree } from "../../../api_helper/TreeWrapper";
import "./SubTreeModal.css";
import { saveSubtree } from "../../../api_helper/TreeWrapper";
import axios from "axios";
import { TreeViewType } from "../../helper/TreeEditorHelper";

const SubtreeModal = ({
  isOpen,
  onClose,
  projectName,
  subtreeName,
  setDiagramEdited,
}: {
  isOpen: boolean;
  onClose: Function;
  projectName: string;
  subtreeName: string;
  setDiagramEdited: React.Dispatch<React.SetStateAction<boolean>>;
}) => {
  // STATE
  const [initialJson, setInitialJson] = useState("");
  const [resultJson, setResultJson] = useState("");
  const [treeStructure, setTreeStructure] = useState("");
  const [view, changeView] = useState<TreeViewType>(TreeViewType.Editor);

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

  // EFFECTS
  useEffect(() => {
    const fetchSubtree = async () => {
      if (isOpen) {
        try {
          const response = await getSubtree(subtreeName, projectName);
          setInitialJson(JSON.parse(response));
          console.log("Initial JSON:", JSON.parse(response));
        } catch (error) {
          console.error("Failed to fetch subtree:", error);
        }
      }
    };

    fetchSubtree();
    getBTTree();
  }, [isOpen, projectName, subtreeName]);

  useEffect(() => {
    setInitialJson(resultJson);
    getBTTree();
    console.log("Changing view!");
  }, [view])

  const handleCancel = async () => {
    try {
      await saveSubtree(resultJson, projectName, subtreeName);
    } catch (error) {
      console.error("Failed to save subtree:", error);
    }
    onClose();
  };

  return (
    <Modal
      open={isOpen}
      onClose={handleCancel}
      aria-labelledby="sub-tree-modal-title"
      aria-describedby="sub-tree-modal-description"
    >
      <Box className="modal-box">
        <div className="modal-menu">
          <h2 className="modal-name">{subtreeName}</h2>
          <IconButton
            aria-label="close"
            onClick={handleCancel}
            className="close-button"
            style={{ color: "white" }}
          >
            <CloseIcon />
          </IconButton>
        </div>
        <div>
          {initialJson && (
            <TreeEditor
              modelJson={initialJson}
              setResultJson={setResultJson}
              projectName={projectName}
              setDiagramEdited={setDiagramEdited}
              hasSubtrees={false}
              treeStructure={treeStructure}
              view={view}
              changeView={changeView}
            />
          )}
        </div>
      </Box>
    </Modal>
  );
};

export default SubtreeModal;
