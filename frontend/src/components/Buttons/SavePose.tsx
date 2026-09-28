import React, { useState } from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useError } from "jderobot-ide-interface";
import { CommsManager, states } from "jderobot-commsmanager";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { LoadingIcon } from "BtIcons";
import PinDropIcon from "@mui/icons-material/PinDropRounded";
import { saveCurrentPose } from "BtApi/TreeWrapper";

interface SavePoseButtonProps {
  project: string;
}

const SavePoseButton = ({ project }: SavePoseButtonProps) => {
  const theme = useBtTheme();
  const { warning, error, info } = useError() as any;
  const [loading, setLoading] = useState<boolean>(false);

  const onSavePose = async () => {
    const manager = CommsManager.getInstance();
    const state = manager.getState();
    const activeUniverse = manager.getUniverse();

    if (!activeUniverse || activeUniverse === "") {
      warning("No active universe loaded. Please select and launch a combined universe first.");
      return;
    }

    if (
      state === states.CONNECTED ||
      state === states.IDLE ||
      state === states.WORLD_READY
    ) {
      warning("Simulation is not running. Please launch the simulation first.");
      return;
    }

    setLoading(true);
    try {
      const pose = await saveCurrentPose(project, activeUniverse);
      const poseStr = `[x:${pose[0]}, y:${pose[1]}, z:${pose[2]}, yaw:${pose[5]}]`;
      if (info) {
        info(`Successfully saved robot's current pose ${poseStr} as the starting pose for universe '${activeUniverse}'!`);
      } else {
        alert(`Successfully saved robot's current pose ${poseStr} as the starting pose for universe '${activeUniverse}'!`);
      }
    } catch (e: any) {
      error(e.response?.data?.message || e.message || "Failed to save the current robot pose. Make sure the robot is active in the simulator.");
    }
    setLoading(false);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.bg}
      hoverColor={theme.palette.primary}
      roundness={theme.roundness}
      id="save-robot-pose"
      onClick={onSavePose}
      title="Save robot current position as starting pose"
      disabled={loading}
    >
      {loading ? (
        <LoadingIcon htmlColor={theme.palette.text} id="loading-spin" />
      ) : (
        <PinDropIcon sx={{ color: theme.palette.text }} />
      )}
    </StyledHeaderButton>
  );
};

export default SavePoseButton;
