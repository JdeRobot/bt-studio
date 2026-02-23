import React, { useState } from "react";
import { CommsManager } from "jderobot-commsmanager";
import {
  ViewersEntry,
  VncViewer,
  contrastSelector,
} from "jderobot-ide-interface";

import TerminalRoundedIcon from "@mui/icons-material/TerminalRounded";
import VideoCameraBackRoundedIcon from "@mui/icons-material/VideoCameraBackRounded";
import TreeMonitorContainer from "BtComponents/TreeMonitor";
import { TreeMonitorIcon } from "BtIcons";
import { useBtTheme } from "BtContexts/BtThemeContext";

const getTools = (manager: CommsManager | null, project: string) => {
  const theme = useBtTheme();
  const [showSim, setSimVisible] = useState<boolean>(true);
  // const [showWebGUI, setWebGUIVisible] = useState<boolean>(true);
  // const [showRviz, setRvizVisible] = useState<boolean>(true);
  const [showTerminal, setTerminalVisible] = useState<boolean>(true);
  const [showMonitor, setMonitorVisible] = useState<boolean>(true);

  const iconColor = contrastSelector(
    theme.palette.text,
    theme.palette.darkText,
    theme.palette.secondary,
  );

  const toolsList: ViewersEntry[] = [];

  toolsList.push({
    component: (
      <TreeMonitorContainer commsManager={manager} project={project} />
    ),
    icon: <TreeMonitorIcon htmlColor={iconColor} />,
    name: "Tree Monitor",
    active: showMonitor,
    activate: setMonitorVisible,
  });

  // if (tools.includes("web_gui")) {
  //   toolsList.push({
  //     component: children,
  //     icon: <ImportantDevicesRoundedIcon />,
  //     name: "Web Gui",
  //     group: "debug-interface",
  //     active: showWebGUI,
  //     activate: setWebGUIVisible,
  //   });
  // }

  // if (tools.includes("simulator")) {
  // }
  toolsList.push({
    component: (
      <VncViewer
        commsManager={manager}
        port={6080}
        message={"Click Play to connect to the Robotics Backend"}
      />
    ),
    icon: <VideoCameraBackRoundedIcon htmlColor={iconColor} />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  });

  // if (tools.includes("rviz")) {
  //   toolsList.push({
  //     component: (
  //       <VncViewer
  //         commsManager={manager}
  //         port={6081}
  //         message={"Click Play to connect to the Robotics Backend"}
  //       />
  //     ),
  //     icon: <PrecisionManufacturingRoundedIcon />,
  //     name: "Rviz",
  //     active: showRviz,
  //     activate: setRvizVisible,
  //   });
  // }

  // if (tools.includes("console")) {
  // }
  toolsList.push({
    component: (
      <VncViewer
        commsManager={manager}
        port={6082}
        message={"Click Play to connect to the Robotics Backend"}
      />
    ),
    icon: <TerminalRoundedIcon htmlColor={iconColor} />,
    name: "Terminal",
    active: showTerminal,
    activate: setTerminalVisible,
  });

  return toolsList;
};

export default getTools;
