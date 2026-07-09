import React, { useState } from "react";
import { useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { LoadingIcon, StopIcon } from "BtIcons";

const TerminateUniverseButton = () => {
  const theme = useBtTheme();
  const { warning } = useError();
  const [loading, setLoading] = useState<boolean>(false);

  const terminateUniverse = async () => {
    const manager = CommsManager.getInstance();

    if (manager.getWorld() === undefined || manager.getWorld() === "") {
      return;
    }

    const state = manager.getState();

    if (state === states.IDLE || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure a world is selected.",
      );
      return;
    }

    setLoading(true);

    if (state === states.RUNNING || state === states.PAUSED) {
      await manager.terminateApplication();
      await manager.terminateTools();
      await manager.terminateWorld();
    } else if (state === states.TOOLS_READY) {
      await manager.terminateTools();
      await manager.terminateWorld();
    } else {
      await manager.terminateWorld();
    }

    setLoading(false);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="stop-world"
      onClick={terminateUniverse}
      title="Stop World"
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
