import { useBtTheme } from "Contexts/BtThemeContext";
import {
  StyledSettingsSection,
  StyledSettingsSectionTitle,
} from "Styles/Modal/Settings/Settings.styles";

const Section = ({ title, children }: { title: string; children: any }) => {
  const theme = useBtTheme();

  return (
    <StyledSettingsSection
      bgColor={theme.palette.background}
      color={theme.palette.text}
    >
      <StyledSettingsSectionTitle>{title}</StyledSettingsSectionTitle>
      {children}
    </StyledSettingsSection>
  );
};

export default Section;
