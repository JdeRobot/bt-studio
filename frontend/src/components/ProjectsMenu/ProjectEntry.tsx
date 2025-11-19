import React, { useState } from "react";
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
import SouthRoundedIcon from "@mui/icons-material/SouthRounded";
import NorthRoundedIcon from '@mui/icons-material/NorthRounded';

import { useError } from "jderobot-ide-interface";
import { deleteProject } from "BtApi/TreeWrapper";
import { publish } from "BtComponents/helper/TreeEditorHelper";

interface Project {
  id: string;
  name: string;
  creator: string;
  last_modified: string;
}

const ProjectEntry = ({ projects }: { projects: Project[] }) => {
  const theme = useBtTheme();
  const [sort, setSort] = useState("last_modified");
  const [order, setOrder] = useState(true); // True = descending / False = ascending

  const entryStyle = {
    bg: theme.palette.bg,
    color: theme.palette.text,
    roundness: theme.roundness,
  };

  const parseTime = (time: string) => {
    const last_date = new Date(time);
    const now = new Date();
    const year_diff = now.getUTCFullYear() - last_date.getUTCFullYear();
    if (year_diff > 0) {
      return `${year_diff > 1 ? year_diff : "a"} year${year_diff > 1 ? "s" : ""} ago`;
    }

    const month_diff = now.getUTCMonth() - last_date.getUTCMonth();
    if (month_diff > 0) {
      return `${month_diff > 1 ? month_diff : "a"} month${month_diff > 1 ? "s" : ""} ago`;
    }

    const day_diff = now.getUTCDate() - last_date.getUTCDate();
    if (day_diff > 0) {
      return `${day_diff > 1 ? day_diff : "a"} day${day_diff > 1 ? "s" : ""} ago`;
    }

    const hour_diff = now.getUTCHours() - last_date.getUTCHours();
    if (hour_diff > 0) {
      return `${hour_diff > 1 ? hour_diff : "an"} hour${hour_diff > 1 ? "s" : ""} ago`;
    }

    const min_diff = now.getUTCMinutes() - last_date.getUTCMinutes();
    if (min_diff > 0) {
      return `${min_diff > 1 ? min_diff : "a"} minute${min_diff > 1 ? "s" : ""} ago`;
    }

    console.log(now, last_date);
    return "now";
  };

  const onSort = (id: string) => {
    if (id === sort) {
      setOrder(!order);
    } else {
      setSort(id);
      setOrder(true);
    }
  };

  if (sort !== undefined) {
    const o1 = order ? -1 : 1;
    const o2 = order ? 1 : -1;
    switch (sort) {
      case "last_modified":
        projects.sort((a, b) =>
          new Date(a[sort]) > new Date(b[sort]) ? o1 : o2
        );
        break;
      case "name":
      case "creator":
        projects.sort((a, b) => (a[sort] < b[sort] ? o1 : o2));
        break;
    }
  }

  return (
    <StyledEntryContainer bg={theme.palette.bg}>
      <StyledEntry {...entryStyle} noHover={true}>
        <SortedButton
          title="Project Name"
          onSort={onSort}
          sort_id="name"
          curr_sort={sort}
          curr_order={order}
        />
        <SortedButton
          title="Creator"
          onSort={onSort}
          sort_id="creator"
          curr_sort={sort}
          curr_order={order}
        />
        <SortedButton
          title="Last modified"
          onSort={onSort}
          sort_id="last_modified"
          curr_sort={sort}
          curr_order={order}
        />
        <h3 style={{textAlign: "end" }}>Actions</h3>
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

const SortedButton = ({
  title,
  onSort,
  sort_id,
  curr_sort,
  curr_order,
}: {
  title: string;
  onSort: (sort_id: string) => void;
  sort_id: string;
  curr_sort: string;
  curr_order: boolean;
}) => {
  const theme = useBtTheme();
  return (
    <h3 style={{ cursor: "pointer" }} onClick={() => onSort(sort_id)}>
      {`${title} `}
      {curr_sort === sort_id && (
        <>
          {curr_order ? (
            <SouthRoundedIcon style={{fontSize: "1rem"}} htmlColor={theme.palette.text} />
          ) : (
            <NorthRoundedIcon style={{fontSize: "1rem"}} htmlColor={theme.palette.text} />
          )}
        </>
      )}
    </h3>
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
