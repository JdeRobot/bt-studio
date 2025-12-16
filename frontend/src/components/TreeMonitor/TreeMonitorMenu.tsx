import React from "react";
import { MouseEventHandler, MutableRefObject } from "react";

import { UndoIcon, ZoomIcon } from "BtIcons";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledMonitorMenu,
  StyledMonitorMenuButton,
  StyledMonitorText,
} from "BtStyles/TreeMonitor/TreeMonitor.styles";

const TreeMonitorMenu = ({
  onZoomToFit,
  setGoBack,
  subTreeName,
}: {
  onZoomToFit: MouseEventHandler;
  setGoBack: (a: boolean) => void;
  subTreeName: MutableRefObject<string>;
}) => {
  const theme = useBtTheme();

  const style = {
    bg: theme.palette.primary,
    roundness: theme.roundness,
  };

  return (
    <StyledMonitorMenu>
      <div style={{ height: "100%", alignContent: "center" }}>
        <StyledMonitorMenuButton
          {...style}
          onClick={onZoomToFit}
          title="Zoom To Fit"
        >
          <ZoomIcon htmlColor={theme.palette.text} />
        </StyledMonitorMenuButton>
        <StyledMonitorMenuButton
          {...style}
          onClick={() => setGoBack(true)}
          title="Go Back"
        >
          <UndoIcon htmlColor={theme.palette.text} />
        </StyledMonitorMenuButton>
      </div>
      <StyledMonitorText color={theme.palette.text}>
        {subTreeName.current}
      </StyledMonitorText>
    </StyledMonitorMenu>
  );
};

export default TreeMonitorMenu;
