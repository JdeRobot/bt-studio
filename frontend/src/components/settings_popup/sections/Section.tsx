import React from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledSettingsSection,
  StyledSettingsSectionTitle,
} from "BtStyles/Modal/Settings/Settings.styles";

const Section = ({ title, children }: { title: string; children: any }) => {
  const theme = useBtTheme();

  return (
    <StyledSettingsSection
      bgColor={theme.palette.bg}
      color={theme.palette.text}
    >
      <StyledSettingsSectionTitle>{title}</StyledSettingsSectionTitle>
      {children}
    </StyledSettingsSection>
  );
};

export default Section;
