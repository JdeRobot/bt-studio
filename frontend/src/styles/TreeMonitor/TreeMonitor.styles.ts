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
  height: 2em;
  display: grid;
  grid-template-columns: auto 2em 2em;
  gap: 0.5rem;
  margin: 0.25rem 1rem;
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

export const StyledContainer = styled.div`
  height: 100%;
  width: 100%;
  position: relative;
  // grid-template-rows: auto 2.5em;
  grid-template-rows: 2.5em auto;
  display: grid;
`;
