import { Link } from "react-router-dom";
import styled from "styled-components";

const primaryColor = "#666";

interface StyledEntryProps {
  bg?: string;
  color?: string;
  roundness?: number;
}

export const StyledEntry = styled.div<StyledEntryProps>`
  background-color: ${(p) => p.bg ?? primaryColor};
  color: ${(p) => p.color ?? primaryColor};

  width: 100%;
  height: 3rem;
  min-height: 3rem;
  padding: 5px 20px;

  display: flex;
  gap: 5px;
  flex-direction: row;
  align-items: center;
  justify-content: space-between;

  &:hover {
    filter: var(--hover-light);
  }

  &:first-of-type {
    border-top-right-radius:${(p) => p.roundness ?? 1}px;
    border-top-left-radius: ${(p) => p.roundness ?? 1}px;
  }

  &:last-of-type {
    border-bottom-right-radius: ${(p) => p.roundness ?? 1}px;
    border-bottom-left-radius: ${(p) => p.roundness ?? 1}px;
  }

  & a {
    text-decoration: none;
    color: inherit;
    align-self: center;
  }
`;

export const StyledSpacer = styled.div<StyledEntryProps>`
  background-color: ${(p) => p.bg ?? primaryColor};
  width: 99%;
  height: 1px;
`;

export const StyledEntryContainer = styled.div<StyledEntryProps>`
  background-color: ${(p) => p.bg ?? primaryColor};
  display: flex;
  flex-direction: column;
  align-items: center;
  border-radius: 5px;
  box-shadow: var(--shadow);

`;

export const StyledActionButton = styled.button<StyledEntryProps>`
  height: 2rem;
  min-height: 2rem;
  border-radius:${(p) => p.roundness ?? 1}px;
  background-color: ${(p) => p.bg ?? primaryColor};
  border:0;

  &:hover {
    filter: var(--hover-strong);
  }
`;

export const StyledActionLink = styled(Link)<StyledEntryProps>`
  height: 2rem;
  min-height: 2rem;
  border-radius:${(p) => p.roundness ?? 1}px;
  background-color: ${(p) => p.bg ?? primaryColor};
  border:0;

  &:hover {
    filter: var(--hover-strong);
  }
`;


export const StyledActionContainer = styled.div<StyledEntryProps>`
  display: flex;
  flex-direction:row;
  gap: 5px;
`;