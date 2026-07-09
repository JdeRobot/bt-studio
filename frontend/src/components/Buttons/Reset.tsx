import React, { useState } from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { LoadingIcon, ResetIcon } from "BtIcons";

const ResetButton = () => {
  const theme = useBtTheme();
  const { warning, error } = useError();
  const [loading, setLoading] = useState<boolean>(false);

  const onResetApp = async () => {
    const manager = CommsManager.getInstance();
    const state = manager.getState();

    if (
      state === states.CONNECTED ||
      state === states.IDLE ||
      state === states.WORLD_READY
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure a world is selected.",
      );
      return;
    }

    if (state === states.TOOLS_READY) {
      return;
    }

    setLoading(true);
    try {
      await manager.terminateApplication();
    } catch {
      error("Failed to reset the application. See the traces in the terminal.");
    }
    setLoading(false);
    console.log("App reseted!");
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="reset-app"
      onClick={onResetApp}
      title="Reset app"
      disabled={loading}
    >
      {loading ? (
        <LoadingIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <ResetIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default ResetButton;
