import React from "react";
import { StyledHeaderButton } from "../../styles/Header/HeaderMenu.styles";
import { useBtTheme } from "../../contexts/BtThemeContext";
import LightModeRoundedIcon from "@mui/icons-material/LightModeRounded";
import DarkModeRoundedIcon from "@mui/icons-material/DarkModeRounded";
import { useState } from "react";

const ThemeButton = () => {
  const theme = useBtTheme();
  const [isDarkTheme, showDarkTheme] = useState<boolean>(
    window.localStorage.getItem("themeType") !== null
      ? window.localStorage.getItem("themeType") === "light"
        ? false
        : true
      : true,
  );

  const switchTheme = () => {
    theme.switch(isDarkTheme ? "light" : "dark");
    showDarkTheme(!isDarkTheme);
  };

  return (
    <StyledHeaderButton
      bgColor={theme.palette.primary}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="theme-button"
      onClick={switchTheme}
      title="Switch Theme"
    >
      {isDarkTheme ? (
        <LightModeRoundedIcon htmlColor={theme.palette.text} />
      ) : (
        <DarkModeRoundedIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default ThemeButton;
