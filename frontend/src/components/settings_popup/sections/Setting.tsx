import React from "react";
import { useBtTheme } from "Contexts/BtThemeContext";
import {
  StyledSetting,
  StyledSettingTitle,
} from "Styles/Modal/Settings/Settings.styles";

const Setting = ({ title, children }: { title: string; children: any }) => {
  const theme = useBtTheme();

  return (
    <StyledSetting color={theme.palette.text}>
      <StyledSettingTitle>{title}</StyledSettingTitle>
      {/* Add settings info */}
      {children}
    </StyledSetting>
  );
};

export default Setting;
