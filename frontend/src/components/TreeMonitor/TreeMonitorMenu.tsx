import React from "react";
import { MouseEventHandler, MutableRefObject } from "react";

import { UndoIcon, ZoomIcon } from "BtIcons";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledMonitorMenu,
  StyledMonitorMenuButton,
  StyledMonitorText,
} from "BtStyles/TreeMonitor/TreeMonitor.styles";
import { contrastSelector } from "jderobot-ide-interface";

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

  const iconColor = contrastSelector(
    theme.palette.text,
    theme.palette.darkText,
    theme.palette.primary,
  );

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
          <ZoomIcon htmlColor={iconColor} />
        </StyledMonitorMenuButton>
        <StyledMonitorMenuButton
          {...style}
          onClick={() => setGoBack(true)}
          title="Go Back"
        >
          <UndoIcon htmlColor={iconColor} />
        </StyledMonitorMenuButton>
      </div>
      <StyledMonitorText color={iconColor}>
        {subTreeName.current}
      </StyledMonitorText>
    </StyledMonitorMenu>
  );
};

export default TreeMonitorMenu;
