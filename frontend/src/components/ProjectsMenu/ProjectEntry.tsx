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

import { Entry, useError } from "jderobot-ide-interface";
import {
  deleteProject,
  generateLocalApp,
  getFile,
  getFileList,
  getProjectConfig,
  getProjectConfigRaw,
} from "BtApi/TreeWrapper";
import { publish } from "BtComponents/helper/TreeEditorHelper";
import JSZip from "jszip";
import TreeGardener from "BtTemplates/TreeGardener";
import RosTemplates from "BtTemplates/RosTemplates";

import {
  CopyIcon,
  DeleteIcon,
  DownArrowIcon,
  DownloadIcon,
  EditIcon,
  UpArrowIcon,
} from "BtIcons";

interface Project {
  id: string;
  name: string;
  creator: string;
  last_modified: string;
}

const ProjectEntry = ({
  projects,
  userFilter,
}: {
  projects: Project[];
  userFilter?: string;
}) => {
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

  let display_proj = projects;

  if (sort !== undefined) {
    const o1 = order ? -1 : 1;
    const o2 = order ? 1 : -1;
    switch (sort) {
      case "last_modified":
        display_proj.sort((a, b) =>
          new Date(a[sort]) > new Date(b[sort]) ? o1 : o2,
        );
        break;
      case "name":
      case "creator":
        display_proj.sort((a, b) => (a[sort] < b[sort] ? o1 : o2));
        break;
    }
    if (userFilter === "You") {
      display_proj = projects.filter((a) => a.creator === "You");
    } else if (userFilter === "shared") {
      display_proj = projects.filter((a) => a.creator !== "You");
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
        <h3 style={{ textAlign: "end" }}>Actions</h3>
      </StyledEntry>
      <StyledSpacer bg={theme.palette.bgLight} />

      {display_proj.map((project, index) => (
        <>
          <StyledEntry {...entryStyle} key={project.id}>
            <Link to={"studio/" + project.id}>{project.name}</Link>
            <label>{project.creator}</label>
            <label>{parseTime(project.last_modified)}</label>
            <Actions project={project.id} />
          </StyledEntry>
          {index < display_proj.length - 1 && (
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
            <DownArrowIcon
              style={{ fontSize: "1rem" }}
              htmlColor={theme.palette.text}
            />
          ) : (
            <UpArrowIcon
              style={{ fontSize: "1rem" }}
              htmlColor={theme.palette.text}
            />
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

  const onDownload = async (): Promise<void> => {
    try {
      // Get the blob from the API wrapper
      const settings = await getProjectConfigRaw(project);
      const appFiles = await generateLocalApp(project, settings.btOrder);

      // Create the zip with the files
      const zip = new JSZip();

      TreeGardener.addLocalFiles(zip);
      RosTemplates.addLocalFiles(
        zip,
        project,
        appFiles.tree,
        appFiles.dependencies,
      );

      const project_dir = zip.folder(project);

      if (project_dir === null) {
        throw Error("Project directory could not be found");
      }

      const file_list = await getFileList(project);

      const files: Entry[] = JSON.parse(file_list);

      let actions = undefined;
      for (const file of files) {
        if (file.is_dir && file.name === "actions") {
          actions = file;
        }
      }

      if (actions === undefined) {
        throw Error("Action directory not found");
      }

      await zipCodeFolder(project_dir, actions);

      zip.generateAsync({ type: "blob" }).then(function (content) {
        // Create a download link and trigger download
        const url = window.URL.createObjectURL(content);
        const a = document.createElement("a");
        a.style.display = "none";
        a.href = url;
        a.download = `${project}.zip`; // Set the downloaded file's name
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url); // Clean up after the download
      });

      console.log("App downloaded successfully");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error downloading app: " + e.message);
        error("Error downloading app: " + e.message);
      }
    }
  };

  const zipCodeFile = async (
    zip: JSZip,
    file_path: string,
    file_name: string,
  ) => {
    const content = await getFile(project, file_path);
    zip.file(file_name, content);
  };

  const zipCodeFolder = async (zip: JSZip, file: Entry) => {
    const folder = zip.folder(file.name);

    if (folder === null) {
      return;
    }

    for (let index = 0; index < file.files.length; index++) {
      const element = file.files[index];
      if (element.is_dir) {
        await zipCodeFolder(folder, element);
      } else {
        await zipCodeFile(folder, element.path, element.name);
      }
    }
  };

  return (
    <StyledActionContainer>
      <Link to={"create_project/" + project}>
        <StyledActionButton
          {...style}
          onClick={() => console.log("Copy project")}
          title="Copy project"
          id="copy-project-button"
        >
          <CopyIcon htmlColor={theme.palette.text} />
        </StyledActionButton>
      </Link>
      <StyledActionButton
        {...style}
        onClick={onDownload}
        title="Download project"
        id="download-project-button"
      >
        <DownloadIcon htmlColor={theme.palette.text} />
      </StyledActionButton>
      {/* <Link to={"edit/" + project}>
        <StyledActionButton
          {...style}
          onClick={() => console.log("edit project")}
          title="Edit project"
          id="edit-project-button"
        >
          <EditIcon htmlColor={theme.palette.text} />
        </StyledActionButton>
      </Link> */}
      <StyledActionButton
        {...style}
        onClick={onDeleteProject}
        title="Delete project"
        id="delete-project-button"
      >
        <DeleteIcon htmlColor={theme.palette.text} />
      </StyledActionButton>
    </StyledActionContainer>
  );
};

export default ProjectEntry;
