import { useBtTheme } from "Contexts/BtThemeContext";
import {
  StyledSettingsSubSection,
  StyledSettingsSubSectionTitle,
} from "Styles/Modal/Settings/Settings.styles";
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
