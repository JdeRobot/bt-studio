import React from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledActionButton,
  StyledActionButtonColor,
  StyledActionContainer,
  StyledEntry,
  StyledEntryContainer,
  StyledSpacer,
} from "BtStyles/ProjectMenu/ProjectEntry.styles";
import { Link } from "react-router-dom";

import FileDownloadRoundedIcon from "@mui/icons-material/FileDownloadRounded";
import DeleteForeverRoundedIcon from "@mui/icons-material/DeleteForeverRounded";
import ContentCopyRoundedIcon from "@mui/icons-material/ContentCopyRounded";
import EditRoundedIcon from "@mui/icons-material/EditRounded";
import { useError } from "jderobot-ide-interface";
import { deleteProject } from "BtApi/TreeWrapper";

const ProjectEntry = ({ projects }: { projects: string[] }) => {
  const theme = useBtTheme();

  const entryStyle = {
    bg: theme.palette.bg,
    color: theme.palette.text,
    roundness: theme.roundness,
  };

  return (
    <StyledEntryContainer bg={theme.palette.bg}>
      <StyledEntry {...entryStyle} style={{ pointerEvents: "none" }}>
        <h3>Project ID</h3>
        <h3 style={{ textAlign: "end" }}>Actions</h3>
      </StyledEntry>
      <StyledSpacer bg={theme.palette.bgLight} />

      {projects.map((project, index) => (
        <>
          <StyledEntry {...entryStyle} key={project}>
            <Link to={"/studio/" + project}>{project}</Link>
            <Actions project={project} />
          </StyledEntry>
          {index < projects.length - 1 && (
            <StyledSpacer bg={theme.palette.bgLight} />
          )}
        </>
      ))}
    </StyledEntryContainer>
  );
};

const Actions = ({ project }: { project: string }) => {
  const theme = useBtTheme();
  const { error } = useError();

  const style = {
    bg: theme.palette.bg,
    roundness: theme.roundness,
  };

  const onDeleteProject = async () => {
    // Delete and update
    try {
      await deleteProject(project);
      console.log("Project deleted successfully");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        error("Error while fetching project list: " + e.message);
      }
    }
  };

  return (
    <StyledActionContainer>
      <Link to="/create_project">
        <StyledActionButton
          {...style}
          onClick={() => console.log("Copy project")}
          title="Copy project"
        >
          <ContentCopyRoundedIcon htmlColor={theme.palette.text} />
        </StyledActionButton>
      </Link>
      <StyledActionButton
        {...style}
        onClick={() => console.log("Download project")}
        title="Download project"
      >
        <FileDownloadRoundedIcon htmlColor={theme.palette.text} />
      </StyledActionButton>
      <Link to={"/edit/" + project}>
        <StyledActionButton
          {...style}
          onClick={() => console.log("edit project")}
          title="Edit project"
        >
          <EditRoundedIcon htmlColor={theme.palette.text} />
        </StyledActionButton>
      </Link>
      <StyledActionButton
        {...style}
        onClick={onDeleteProject}
        title="Delete project"
      >
        <DeleteForeverRoundedIcon htmlColor={theme.palette.text} />
      </StyledActionButton>
    </StyledActionContainer>
  );
};

export default ProjectEntry;
