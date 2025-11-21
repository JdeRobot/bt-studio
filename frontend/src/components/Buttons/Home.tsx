import React from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { Link } from "react-router-dom";
import { HomeIcon } from "BtIcons";

const HomeButton = () => {
  const theme = useBtTheme();

  return (
    <Link to="/projects">
      <StyledHeaderButton
        bgColor={theme.palette.bg}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id="return-academy"
        title="Return to Home"
      >
        <HomeIcon htmlColor={theme.palette.text} />
      </StyledHeaderButton>
    </Link>
  );
};

export default HomeButton;
