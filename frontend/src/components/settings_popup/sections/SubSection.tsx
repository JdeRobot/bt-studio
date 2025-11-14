import React from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledSettingsSubSection,
  StyledSettingsSubSectionTitle,
} from "BtStyles/Modal/Settings/Settings.styles";
const SubSection = ({ title, children }: { title: string; children: any }) => {
  const theme = useBtTheme();

  return (
    <StyledSettingsSubSection color={theme.palette.text}>
      <StyledSettingsSubSectionTitle>{title}</StyledSettingsSubSectionTitle>
      {children}
    </StyledSettingsSubSection>
  );
};

export default SubSection;
