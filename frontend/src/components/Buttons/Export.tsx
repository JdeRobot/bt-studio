import React from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { useEffect, useRef, useState } from "react";
import { subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import { generateDockerizedApp, getFile, getFileList } from "BtApi/TreeWrapper";
import JSZip from "jszip";
import TreeGardener from "BtTemplates/TreeGardener";
import RosTemplates from "BtTemplates/RosTemplates";
import { Entry, useError } from "jderobot-ide-interface";
import { ExportIcon } from "BtIcons";
import { useProjectSettings } from "BtContexts/ProjectSettingsContext";

const ExportButton = ({ project }: { project: string }) => {
  const theme = useBtTheme();
  const settings = useProjectSettings();
  const { error } = useError();
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  useEffect(() => {
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });

    return () => {
      unsubscribe("autoSaveCompleted", () => {});
    };
  }, []);

  const saveFile = async (save?: boolean): Promise<any> => {
    // TODO: add later
    // if (save === undefined) {
    //   publish("autoSave");
    //   updateCode(false);
    // }

    // if (!isCodeUpdatedRef.current) {
    //   console.log("Try autosave", isCodeUpdated);
    //   return setTimeout(saveFile, 100, true);
    // }

    try {
      // Get the blob from the API wrapper
      const appFiles = await generateDockerizedApp(
        project,
        settings.btOrder.value,
      );

      // Create the zip with the files
      const zip = new JSZip();

      zip.file("self_contained_tree.xml", appFiles.tree);
      TreeGardener.addDockerFiles(zip);
      RosTemplates.addDockerFiles(zip);

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

      await zipCodeFolder(zip, actions);

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
    <StyledHeaderButton
      bgColor={theme.palette.bg}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="export-code"
      onClick={() => saveFile(undefined)}
      title="Export project to another JdeRobot app"
    >
      <ExportIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default ExportButton;
