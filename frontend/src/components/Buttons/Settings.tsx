import { StyledHeaderButton } from "../../styles/headers/HeaderMenu.styles";
import { useBtTheme } from "../../contexts/BtThemeContext";
import SettingsRoundedIcon from "@mui/icons-material/SettingsRounded";
import { useState } from "react";
import SettingsModal from "../settings_popup/SettingsModal";

const SettingsButton = ({ project }: { project: string }) => {
  const theme = useBtTheme();
  const [isSettingsModalOpen, setSettingsModalOpen] = useState(false);

  const onOpenSettingsModal = (e: any) => {
    setSettingsModalOpen(true);
  };

  const onCloseSettingsModal = () => {
    setSettingsModalOpen(false);
  };

  return (
    <>
      <SettingsModal
        isOpen={isSettingsModalOpen}
        onSubmit={(data: unknown) => {}}
        onClose={onCloseSettingsModal}
        currentProjectname={project}
      />
      <StyledHeaderButton
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id="open-settings-manager"
        onClick={onOpenSettingsModal}
        title="Settings"
      >
        <SettingsRoundedIcon htmlColor={theme.palette.text} />
      </StyledHeaderButton>
    </>
  );
};

export default SettingsButton;
