import React, { useEffect, useRef, useState } from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { Entry, useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import JSZip from "jszip";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { generateDockerizedApp, getFileList } from "BtApi/TreeWrapper";
import TreeGardener from "BtTemplates/TreeGardener";
import RosTemplates from "BtTemplates/RosTemplates";
import { publish, subscribe, unsubscribe, zipCodeFiles } from "BtHelpers/utils";
import { LoadingIcon, PauseIcon, PlayIcon } from "BtIcons";
import { useProjectSettings } from "BtContexts/ProjectSettingsContext";

const PlayPauseButton = ({ project }: { project: string }) => {
  const settings = useProjectSettings();
  const theme = useBtTheme();
  const { warning, error } = useError();
  const filesRef = useRef<Entry[]>([]);
  const runningFilesRef = useRef<JSZip>(JSZip);
  const [state, setState] = useState<string>(states.IDLE);
  const [loading, setLoading] = useState<boolean>(false);
  const isCodeUpdatedRef = useRef<boolean | undefined>(undefined);
  const [, _updateCode] = useState<boolean | undefined>(false);

  const updateCode = (data?: boolean) => {
    isCodeUpdatedRef.current = data;
    _updateCode(data);
  };

  const updateState = (e: unknown) => {
    const T = CustomEvent<{ detail: unknown }>;
    if (e instanceof T) {
      setState(e.detail.state);
    }
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

  const compareZips = async (zip1: JSZip, zip2: JSZip) => {
    for (const key in zip1.files) {
      if (!Object.hasOwn(zip1.files, key)) continue;
      if (!Object.hasOwn(zip2.files, key)) {
        return false;
      }

      const value = await zip1.files[key]._data;
      const old = await zip2.files[key]._data;
      if (value !== old) {
        return false;
      }
    }
    return true;
  };

  const mergeZips = async (zip1: JSZip, zip2: JSZip) => {
    let mergeZip = new JSZip();
    for (const zipObject of [zip1, zip2]) {
      mergeZip = await mergeZip.loadAsync(
        await zipObject.generateAsync({ type: "blob" }),
        { createFolders: true },
      );
    }
    return mergeZip;
  };

  // App handling

  const onAppStateChange = async (save?: boolean): Promise<void> => {
    const manager = CommsManager.getInstance();
    const state = manager.getState();

    setLoading(true);

    if (state === states.WORLD_READY || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure a world is selected.",
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
          "Failed to stop the application. See the traces in the terminal.",
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
      setTimeout(onAppStateChange, 100, true);
      return;
    }

    const files = await getFileList(project);
    filesRef.current = JSON.parse(files);
    const userZip = await loadFiles(filesRef.current);

    if (state === states.PAUSED) {
      const sameZips = await compareZips(userZip, runningFilesRef.current);
      if (sameZips) {
        try {
          await manager.resume();
          console.log("App resumed correctly!");
        } catch (e: unknown) {
          console.error("Error resuming app: " + (e as Error).message);
          error(
            "Failed to resume the application. See the traces in the terminal.",
          );
        }
        setLoading(false);
        return;
      }
    }

    try {
      runningFilesRef.current = userZip;
      const helperZip = new JSZip();
      // Get the blob from the API wrapper
      const appFiles = await generateDockerizedApp(
        project,
        settings.btOrder.value,
      );
      helperZip.file("self_contained_tree.xml", appFiles.tree);
      TreeGardener.addDockerFiles(helperZip);
      RosTemplates.addDockerFiles(helperZip);

      const finalZip = await mergeZips(helperZip, userZip);

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
              base64data as string,
            );
          } catch {
            error(
              "Failed to run the application. See the traces in the terminal.",
            );
            setLoading(false);
          }
          console.log("Dockerized app started successfully");
        }
      };

      finalZip.generateAsync({ type: "blob" }).then(function (content: Blob) {
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

    async function loadFiles(files: Entry[]) {
      const zip = new JSZip();

      let actions = undefined;
      for (const file of filesRef.current) {
        if (file.is_dir && file.name === "actions") {
          actions = file;
        }
      }

      if (actions === undefined) {
        throw Error("Action directory not found");
      }

      await zipCodeFiles(zip, files, project);
      return zip;
    }
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="run-app"
      onClick={() => onAppStateChange(undefined)}
      title="Run app"
      disabled={loading}
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
