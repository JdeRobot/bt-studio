import { StyledHeaderButton } from "../../styles/headers/HeaderMenu.styles";
import HelpRoundedIcon from "@mui/icons-material/HelpRounded";
import { useBtTheme } from "../../contexts/BtThemeContext";

const DocumentationButton = () => {
  const theme = useBtTheme();

  const openInNewTab = (url: URL) => {
    const newWindow = window.open(url, "_blank");
    if (newWindow) {
      newWindow.focus();
    } else {
      console.error("Failed to open new tab/window.");
    }
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="reset-app"
      onClick={() => {
        openInNewTab(
          new URL("https://jderobot.github.io/bt-studio/documentation/"),
        );
      }}
      title="Go to forum"
    >
      <HelpRoundedIcon htmlColor={theme.palette.text} />
    </StyledHeaderButton>
  );
};

export default DocumentationButton;
