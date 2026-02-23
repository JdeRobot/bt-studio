import React, { useState, useEffect, useRef, MutableRefObject } from "react";
import HeaderMenu from "BtComponents/HeaderMenu";
import { CommsManager } from "jderobot-commsmanager";
import { getProjectConfig, saveProjectConfig } from "BtApi/TreeWrapper";

import IdeInterface, { useError } from "jderobot-ide-interface";
import TreeEditorContainer, {
  AddSubtreeButton,
  BTSelectorButtons,
  OtherButtons,
} from "BtComponents/TreeEditor";
import { explorers } from "BtComponents/Explorers";
import { statusBar } from "BtComponents/StatusBar";
import { editorApi } from "BtComponents/Editors";

import { useParams } from "react-router-dom";
import { StyledAppContainer } from "BtStyles/App.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  ProjectSettingsProvider,
  useProjectSettings,
} from "BtContexts/ProjectSettingsContext";
import getTools from "BtComponents/helper/tools";

export const clearTimeouts = (
  timeoutsRef: MutableRefObject<number | null>[],
) => {
  for (const element of timeoutsRef) {
    if (element.current) {
      window.clearTimeout(element.current);
    }
  }
};

const Wrapper = () => {
  const { proj_id } = useParams();
  const { error_critical } = useError();
  const hasUserLimit = window.location.href.includes("unibotics");
  const maxUsers = 15;
  const currentUsers = useRef<number>(0);
  const btAtMaxCapacity = useRef<boolean>(false);

  if (proj_id === undefined) {
    return <></>;
  }

  //Only needed in Unibotics

  useEffect(() => {
    console.log("Current number of users connected: " + currentUsers.current);
    addUser();
    console.log(
      "Now the updated value of users connected is: ",
      currentUsers.current,
    );
    console.log(
      "Current value of UsersAtMaxCapacity: ",
      btAtMaxCapacity.current,
    );
    updateBtAtMaxCapacity(currentUsers.current);
    console.log(
      "Updated value of UsersAtMaxCapacity: ",
      btAtMaxCapacity.current,
    );

    return () => {
      substractUser();
    };
  }, []);

  /////////////////////////////Functions only used in Unibotics/////////////////

  const addUser = () => {
    currentUsers.current += 1;
  };

  const substractUser = () => {
    currentUsers.current -= 1;
  };

  const updateBtAtMaxCapacity = (currentUserCount: number) => {
    console.log("Entering update of MaxCapacity");
    if (currentUserCount > maxUsers && hasUserLimit) {
      console.log("Too much users!");
      btAtMaxCapacity.current = true;
      error_critical(
        "There's not enough room for you to enter Studio. Please try again later.",
        "../..",
      );
    } else {
      console.log("The user can go in");
      btAtMaxCapacity.current = false;
    }
  };

  return (
    <ProjectSettingsProvider>
      <App projectId={proj_id} />
    </ProjectSettingsProvider>
  );
};

const App = ({ projectId }: { projectId: string }) => {
  const theme = useBtTheme();
  const settings = useProjectSettings();
  const hasTriedToConnect = useRef<boolean>(false);
  const timeoutRef = useRef<number | null>(null);
  const connectTimeoutRef = useRef<number | null>(null);
  const [manager, setManager] = useState<CommsManager | null>(null);
  const toolsList = getTools(manager, projectId);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both",
  );

  const saveSettings = async (project: string) => {
    const json_settings: { name: string; config: { [id: string]: any } } = {
      name: project,
      config: {},
    };

    Object.entries(settings).map(([key, setting]) => {
      json_settings.config[key] = setting.value;
    });

    console.log("Save: ", project);

    try {
      await saveProjectConfig(project, JSON.stringify(json_settings));
    } catch (e) {
      console.error("Error saving config:", e);
    }
  };

  // RB manager setup

  useEffect(() => {
    const manager = CommsManager.getInstance();
    setManager(manager);
    getProjectConfig(projectId, settings);

    return () => {
      if (hasTriedToConnect.current) {
        const currManager = CommsManager.getInstance();
        if (currManager) {
          currManager.disconnect();
          CommsManager.deleteInstance();
          setManager(null);
        }
      }
      clearTimeouts([timeoutRef, connectTimeoutRef]);
      saveSettings(projectId);
    };
  }, []);

  const connectWithRetry = async (
    desiredState?: string,
    callback?: () => void,
  ) => {
    try {
      const currManager = CommsManager.getInstance();
      hasTriedToConnect.current = true;
      await currManager.connect();
      console.log("Connected!", currManager.getState());
      setManager(currManager);
      if (callback) {
        waitManagerState(desiredState ? desiredState : "connected", callback);
      }
    } catch {
      console.log("Connection failed, trying again!");
      timeoutRef.current = window.setTimeout(
        connectWithRetry,
        2000,
        desiredState,
        callback,
      );
    }
  };

  const waitManagerState = async (state: string, callback: () => void) => {
    const currManager = CommsManager.getInstance();
    if (currManager?.getState() === state) {
      callback();
    } else {
      connectTimeoutRef.current = window.setTimeout(
        waitManagerState,
        100,
        state,
        callback,
      );
    }
  };

  const treeEditor = {
    component: TreeEditorContainer,
    buttons: [
      <BTSelectorButtons key="BTSelectorButtons" project={projectId} />,
      <AddSubtreeButton key="AddSubtreeButton" project={projectId} />,
      <OtherButtons key="OtherButtons" />,
    ],
    name: "Tree editor",
    language: "custom_tree_editor",
    trigger: [{ group: "Trees", extension: "json" }],
  };

  return (
    <StyledAppContainer bg={theme.palette.bg} hoverStyle="lighten">
      <HeaderMenu
        projectId={projectId}
        connectManager={connectWithRetry}
        setLayout={setLayout}
      />
      <IdeInterface
        commsManager={manager}
        connectManager={connectWithRetry}
        project={projectId}
        explorers={explorers}
        api={editorApi}
        extraEditors={[treeEditor]}
        viewers={toolsList}
        options={[]}
        layout={layout}
        statusBarComponents={statusBar}
      />
    </StyledAppContainer>
  );
};

export default Wrapper;
