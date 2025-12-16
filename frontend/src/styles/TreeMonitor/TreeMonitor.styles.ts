import styled from "styled-components";

const primaryColor = "#666";

interface StyledMonitorMenuButtonProps {
  bg?: string;
  color?: string;
  roundness?: number;
  noHover?: boolean;
}

export const StyledMonitorMenuButton = styled.button<StyledMonitorMenuButtonProps>`
  height: 2rem;
  min-height: 2rem;
  border-radius: ${(p) => p.roundness ?? 1}px;
  background-color: ${(p) => p.bg ?? primaryColor};
  border: 0;
  gap: 5%;

  &:hover {
    filter: var(--hover-strong);
  }
`;

export const StyledMonitorMenu = styled.div`
  display: flex;
  flex-direction: row;
  height: 3em;  
`;

interface StyledMonitorTextProps {
  color?: string;
}

export const StyledMonitorText = styled.h2<StyledMonitorTextProps>`
  font-size: 18px;
  margin: 0;
  align-content: center;
  color: ${(p) => p.color ?? primaryColor};
`;