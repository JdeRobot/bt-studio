import styled from "styled-components";

const primaryColor = "#666";

interface StyledLimitIndicatorProps {
  bg?: string;
  width?: number;
}

export const StyledLimitIndicator = styled.div<StyledLimitIndicatorProps>`
  height: 4px;
  width: ${(p) => p.width ?? 0}%;
  border-radius: 5px;
  background-color: ${(p) => p.bg ?? primaryColor};
`;

interface StyledLimitIndicatorContainerProps {
  bg?: string;
}

export const StyledLimitIndicatorContainer = styled.div<StyledLimitIndicatorContainerProps>`
  height: 4px;
  width: 100%;
  border-radius: 5px;
  overflow: hidden;
  background-color: ${(p) => p.bg ?? primaryColor};
`;

export const StyledIndicatorContainer = styled.div`
  height: 2rem;
  width: 100%;
  overflow: hidden;
`;

interface StyledLimitIndicatorTextProps {
  color?: string;
  width: number;
  full_bg?: string;
}

const handleFull = (p: StyledLimitIndicatorTextProps) => {
  if (p.width > 80) {
    return `color: ${p.full_bg ?? primaryColor};`
  }
};


export const StyledLimitIndicatorText = styled.div<StyledLimitIndicatorTextProps>`
  color: ${(p) => p.color ?? primaryColor};
  font-size: 0.85rem;
  margin-top: 0.5rem;

  &:hover {
    filter: var(--hover-strong);

    & #base {
      opacity: 60%;
    }

    & #indic {
      opacity: 100%;
    }
  }

  & #base {
    opacity: 40%;
  }

  & #indic {
    opacity: 60%;
    font-weight: bolder;
    ${handleFull}
  }
`;