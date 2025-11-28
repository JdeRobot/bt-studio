import React from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { Entry, useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import JSZip from "jszip";
import { useEffect, useRef, useState } from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";

import { generateDockerizedApp, getFile, getFileList } from "BtApi/TreeWrapper";
import TreeGardener from "BtTemplates/TreeGardener";
import RosTemplates from "BtTemplates/RosTemplates";
import { publish, subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import { LoadingIcon, PauseIcon, PlayIcon } from "BtIcons";
import { useProjectSettings } from "BtContexts/ProjectSettingsContext";

const PlayPauseButton = ({
  project,
  connectManager,
}: {
  project: string;
  connectManager: (
    desiredState?: string,
    callback?: () => void
  ) => Promise<void>;
}) => {
  const settings = useProjectSettings();
  const theme = useBtTheme();
  const { warning, error, info, close } = useError();
  const codeRef = useRef("");
  const runningCodeRef = useRef("");
  const [state, setState] = useState<string>(
    CommsManager.getInstance().getState()
  );
  const [loading, setLoading] = useState<boolean>(false);
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [isCodeUpdated, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  const updateState = (e: any) => {
    setState(e.detail.state);
  };

  useEffect(() => {
    subscribe("autoSaveCompleted", () => {
      updateCode(true);
    });
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("autoSaveCompleted", () => {});
      unsubscribe("CommsManagerStateChange", () => {});
    };
  }, []);

  useEffect(() => {
    if (
      state === states.RUNNING ||
      state === states.PAUSED ||
      state === states.TOOLS_READY
    ) {
      setLoading(false);
    }
  }, [state]);

  // App handling

  const onAppStateChange = async (save?: boolean): Promise<any> => {
    const manager = CommsManager.getInstance();
    const state = manager.getState();

    setLoading(true);

    if (state === states.IDLE) {
      info("Connecting with the Robotics Backend ...");
      connectManager(states.CONNECTED, () => {
        setLoading(false);
        close();
        onAppStateChange();
      });
      return;
    }

    if (state === states.WORLD_READY || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected."
      );
      setLoading(false);
      return;
    }


    if (state === states.RUNNING) {
      try {
        await manager.pause();
        console.log("App paused correctly!");
      } catch (e: unknown) {
        console.error("Error pausing app: " + (e as Error).message);
        error(
          "Failed to stop the application. See the traces in the terminal."
        );
      }
      setLoading(false);
      return;
    }

    if (save === undefined) {
      publish("autoSave");
      updateCode(false);
    }

    if (!isCodeUpdatedRef.current) {
      return setTimeout(onAppStateChange, 100, true);
    }

    if (state === states.PAUSED && runningCodeRef.current === codeRef.current) {
      try {
        await manager.resume();
        console.log("App resumed correctly!");
      } catch (e: unknown) {
        console.error("Error resuming app: " + (e as Error).message);
        error(
          "Failed to resume the application. See the traces in the terminal."
        );
      }
      setLoading(false);
      return;
    }

    try {
      // Get the blob from the API wrapper
      const appFiles = await generateDockerizedApp(
        project,
        settings.btOrder.value
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
          try {
            await manager.run(
              "/workspace/code/execute_docker.py",
              ["actions/*.py"],
              base64data as string
            );
            console.log("Dockerized app started successfully");
          } catch (e:unknown) {
            error(
              "Failed to run the application. See the traces in the terminal."
            );
            setLoading(false);
          }
        }
      };

      zip.generateAsync({ type: "blob" }).then(function (content: Blob) {
        reader.readAsDataURL(content);
      });

      console.log("App started successfully");
    } catch (e: unknown) {
      setLoading(false);
      if (e instanceof Error) {
        console.error("Error running app: " + e.message);
        error("Error running app: " + e.message);
      }
    }
  };

  const zipCodeFile = async (
    zip: JSZip,
    file_path: string,
    file_name: string
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
      hoverColor={theme.palette.primary}
      roundness={theme.roundness}
      id="run-app"
      onClick={() => onAppStateChange(undefined)}
      title="Run app"
    >
      {loading ? (
        <LoadingIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <>
          {state === states.RUNNING ? (
            <PauseIcon htmlColor={theme.palette.text} />
          ) : (
            <PlayIcon htmlColor={theme.palette.text} />
          )}
        </>
      )}
    </StyledHeaderButton>
  );
};

export default PlayPauseButton;
