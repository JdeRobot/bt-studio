import React, { useState } from "react";
import { useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import StopCircleRoundedIcon from "@mui/icons-material/StopCircleRounded";
import SyncRoundedIcon from "@mui/icons-material/SyncRounded";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { LoadingIcon, StopIcon } from "BtIcons";

const TerminateUniverseButton = () => {
  const theme = useBtTheme();
  const { warning } = useError();
  const [loading, setLoading] = useState<boolean>(false);

  const terminateUniverse = async () => {
    const manager = CommsManager.getInstance();

    if (manager.getUniverse() === undefined || manager.getUniverse() === "") {
      return;
    }

    const state = manager.getState();

    if (state === states.IDLE || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected.",
      );
      return;
    }

    setLoading(true);

    if (state === states.RUNNING || state === states.PAUSED) {
      await manager.terminateApplication();
      await manager.terminateTools();
      await manager.terminateUniverse();
    } else if (state === states.TOOLS_READY) {
      await manager.terminateTools();
      await manager.terminateUniverse();
    } else {
      await manager.terminateUniverse();
    }

    setLoading(false);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.bg}
      hoverColor={theme.palette.primary}
      roundness={theme.roundness}
      id="stop-universe"
      onClick={terminateUniverse}
      title="Stop Universe"
      disabled={loading}
    >
      {loading ? (
        <LoadingIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <StopIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default TerminateUniverseButton;
