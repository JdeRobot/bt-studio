import styled from "styled-components";

const primaryColor = "#666";

interface StyledHomeContainerProps {
  bg?: string;
}

export const StyledProjectsContainer = styled.div<StyledHomeContainerProps>`
  flex-grow: 1;
  border-radius: 25px 0 0 0;
  background-color: ${(p) => p.bg ?? primaryColor};
  box-shadow: var(--shadow);
`;


interface StyledSectionNameProps {
  color?: string;
}

export const StyledSectionName = styled.h1<StyledSectionNameProps>`
  color: ${(p) => p.color ?? primaryColor};
`;