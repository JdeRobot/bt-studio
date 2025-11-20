import React from "react";
import { StyledHeaderButton } from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { useState } from "react";
import { DarkModeIcon, LightModeIcon } from "BtIcons";

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
      bgColor={theme.palette.bg}
      hoverColor={theme.palette.secondary}
      roundness={theme.roundness}
      id="theme-button"
      onClick={switchTheme}
      title="Switch Theme"
    >
      {isDarkTheme ? (
        <LightModeIcon htmlColor={theme.palette.text} />
      ) : (
        <DarkModeIcon htmlColor={theme.palette.text} />
      )}
    </StyledHeaderButton>
  );
};

export default ThemeButton;
