import React, { useState } from "react";
import { StyledHeaderConnectButton } from "BtStyles/Header/HeaderMenu.styles";
import { states } from "jderobot-commsmanager";
import { useBtTheme } from "BtContexts/BtThemeContext";

const ConnectButton = ({
  connectManager,
}: {
  connectManager: (
    desiredState?: string,
    callback?: () => void,
  ) => Promise<void>;
}) => {
  const theme = useBtTheme();
  const [loading, setLoading] = useState<boolean>(false);

  const handleConnect = async () => {
    setLoading(true);
    connectManager(states.CONNECTED, () => {
      setLoading(false);
      close();
    });
  };

  const msg = loading ? "Conecting ..." : "Click to connect";

  return (
    <StyledHeaderConnectButton
      color={theme.palette.text}
      bgColor={theme.palette.secondary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="connect-with-rb"
      onClick={handleConnect}
      title="Connect to the Robotics Backend"
      disabled={loading}
    >
      {msg}
    </StyledHeaderConnectButton>
  );
};

export default ConnectButton;
