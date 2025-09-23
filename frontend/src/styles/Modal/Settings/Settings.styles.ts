import styled from "styled-components";

const primaryColor = "#666";

interface StyledSettingsListConatinerProps {
  scrollbar?: string;
}

export const StyledSettingsListConatiner = styled.ul<StyledSettingsListConatinerProps>`
  width: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 0px;
  overflow-y: auto;
  max-height: 65vh;
  flex: 1;

  &::-webkit-scrollbar {
    width: 5px;
  }

  &::-webkit-scrollbar-track {
    box-shadow: inset 0 0 5px ${(p) => p.scrollbar ?? primaryColor};
    border-radius: 10px;
  }

  &::-webkit-scrollbar-thumb {
    background: ${(p) => p.scrollbar ?? primaryColor};
    border-radius: 10px;
  }
`;

interface StyledSettingsSectionProps {
  color?: string;
  bgColor?: string;
}

export const StyledSettingsSection = styled.div<StyledSettingsSectionProps>`
  display: inline-table;
  width: 75%;
  min-height: 3rem;
  background-color: ${(p) => p.bgColor ?? primaryColor};
  padding: 5px;
  align-content: center;
  color: ${(p) => p.color ?? primaryColor};
`;

export const StyledSettingsSectionTitle = styled.label`
  margin-top: 10px;
  background-color: transparent !important;
  margin-bottom: 0px !important;
  font-size: larger;
`;

interface StyledSettingsSubSectionProps {
  color?: string;
}

export const StyledSettingsSubSection = styled.div<StyledSettingsSubSectionProps>`
  width: 100%;
  min-height: 3rem;
  margin-top: 5px;
  margin-left: 10px;
  padding: 5px;
  align-content: center;
  color: ${(p) => p.color ?? primaryColor};
`;

export const StyledSettingsSubSectionTitle = styled.label`
  margin-bottom: 0px !important;
  font-size: large;
`;

interface StyledSettingProps {
  color?: string;
}

export const StyledSetting = styled.div<StyledSettingProps>`
  width: auto;
  min-height: 3rem;
  margin-left: 10px;
  margin-top: 5px;
  padding: 5px;
  align-content: center;
  justify-content: left;
  display: flex;
  gap: 5px;
  flex-direction: column;
  color: ${(p) => p.color ?? primaryColor};
`;

export const StyledSettingTitle = styled.label`
  margin-bottom: 0px !important;
  font-size: medium;
`;