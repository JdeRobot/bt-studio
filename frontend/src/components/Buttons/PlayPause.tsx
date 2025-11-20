import React from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { Entry, useError } from "jderobot-ide-interface";
import { CommsManager } from "jderobot-commsmanager";
import JSZip from "jszip";
import { useContext, useEffect, useRef, useState } from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";

import { generateDockerizedApp, getFile, getFileList } from "BtApi/TreeWrapper";
import TreeGardener from "BtTemplates/TreeGardener";
import RosTemplates from "BtTemplates/RosTemplates";
import { OptionsContext } from "../options/Options";
import { subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import { PauseIcon, PlayIcon } from "BtIcons";

const PlayPauseButton = ({
  project,
  manager,
  appRunning,
  setAppRunning,
}: {
  project: string;
  manager: CommsManager | null;
  appRunning: boolean;
  setAppRunning: Function;
}) => {
  const settings = useContext(OptionsContext);
  const theme = useBtTheme();
  const { warning, error } = useError();
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

  // App handling

  const onAppStateChange = async (save?: boolean): Promise<any> => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      );
      return;
    }

    if (
      manager.getState() !== "tools_ready" &&
      manager.getState() !== "application_running" &&
      manager.getState() !== "paused"
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected.",
      );
      return;
    }

    if (appRunning) {
      try {
        await manager.pause();
        setAppRunning(false);
        console.log("App paused correctly!");
        return;
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Error pausing app: " + e.message);
          error("Error pausing app: " + e.message);
        }
      }
    }

    // TODO: add later
    // if (save === undefined) {
    //   publish("autoSave");
    //   updateCode(false);
    // }

    // if (!isCodeUpdatedRef.current) {
    //   return setTimeout(onAppStateChange, 100, true);
    // }

    // if (
    //   manager.getState() === "paused" &&
    //   runningCodeRef.current === codeRef.current
    // ) {
    //   await manager.resume();
    //   setAppRunning(true);
    //   console.log("App resumed correctly!");
    //   return;
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

      // Convert the blob to base64 using FileReader
      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64data = reader.result; // Get the zip in base64
        // Send the base64 encoded blob
        if (base64data) {
          await manager.run(
            "/workspace/code/execute_docker.py",
            ["actions/*.py"],
            base64data as string,
          );
          console.log("Dockerized app started successfully");
        }
      };

      zip.generateAsync({ type: "blob" }).then(function (content: Blob) {
        reader.readAsDataURL(content);
      });

      setAppRunning(true);
      console.log("App started successfully");
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Error running app: " + e.message);
        error("Error running app: " + e.message);
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
      id="run-app"
      onClick={() => onAppStateChange(undefined)}
      title="Run app"
    >
      {appRunning ? (
        <PauseIcon htmlColor={theme.palette.text} />
      ) : (
        <PlayIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default PlayPauseButton;
