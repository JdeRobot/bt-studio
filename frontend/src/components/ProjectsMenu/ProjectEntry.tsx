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
import { publish } from "BtComponents/helper/TreeEditorHelper";

interface Project {
  id: string,
  name: string,
  creator: string,
  last_modified: string,
}

const ProjectEntry = ({ projects }: { projects: Project[] }) => {
  const theme = useBtTheme();

  const entryStyle = {
    bg: theme.palette.bg,
    color: theme.palette.text,
    roundness: theme.roundness,
  };

  const parseTime = (time: string) => {
    const last_date = new Date(time);
    const now = new Date();
    const year_diff = now.getUTCFullYear() - last_date.getUTCFullYear()
    if (year_diff > 0) {
      return `${year_diff > 1 ? year_diff : "a"} year${year_diff > 1 ? "s" : ""} ago`
    }

    const month_diff = now.getUTCMonth() - last_date.getUTCMonth()
    if (month_diff > 0) {
      return `${month_diff > 1 ? month_diff : "a"} month${month_diff > 1 ? "s" : ""} ago`
    }

    const day_diff = now.getUTCDate() - last_date.getUTCDate()
    if (day_diff > 0) {
      return `${day_diff > 1 ? day_diff : "a"} day${day_diff > 1 ? "s" : ""} ago`
    }

    const hour_diff = now.getUTCHours() - last_date.getUTCHours()
    if (hour_diff > 0) {
      return `${hour_diff > 1 ? hour_diff : "an"} hour${hour_diff > 1 ? "s" : ""} ago`
    }

    const min_diff = now.getUTCMinutes() - last_date.getUTCMinutes()
    if (min_diff > 0) {
      return `${min_diff > 1 ? min_diff : "a"} minute${min_diff > 1 ? "s" : ""} ago`
    }

    console.log(now, last_date)
    return "now"
  }


  projects.sort((a, b) => (new Date(a['last_modified']) > new Date(b['last_modified']) ? -1 : 1));
  // projects.sort((a, b) => (new Date(a.last_modified) < new Date(b.last_modified) ? -1 : 1));

  return (
    <StyledEntryContainer bg={theme.palette.bg}>
      <StyledEntry {...entryStyle} style={{ pointerEvents: "none" }}>
        <h3>Project Name</h3>
        <h3>Creator</h3>
        <h3>Last modified</h3>
        <h3 style={{ textAlign: "end" }}>Actions</h3>
      </StyledEntry>
      <StyledSpacer bg={theme.palette.bgLight} />

      {projects.map((project, index) => (
        <>
          <StyledEntry {...entryStyle} key={project.id}>
            <Link to={"/studio/" + project.id}>{project.name}</Link>
            <label>{project.creator}</label>
            <label>{parseTime(project.last_modified)}</label>
            <Actions project={project.id} />
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
      publish("updateProjects");
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
      <Link to={"/create_project/" + project}>
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
