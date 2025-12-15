import { Link } from "react-router-dom";
import styled from "styled-components";

const primaryColor = "#666";

interface StyledHomeContainerProps {
  bg?: string;
}

export const StyledHomeContainer = styled.div<StyledHomeContainerProps>`
  display: flex;
  flex-direction: row;
  height: 100%;
  flex-grow: 1;
  background-color: ${(p) => p.bg ?? primaryColor};
`;

export const StyledActionsContainer = styled.div`
  width: 15%;
  display: flex;
  flex-direction: column;
  align-items: center;
`;

export const StyledActionsSection = styled.div`
  width: 100%;
  display: flex;
  flex-direction: column;
  margin-top: 20px;
  align-items: center;
`;

export const StyledActionsSubSection = styled.div`
  display: flex;
  flex-direction: column;
  width: 90%;
  justify-content: center;
  gap: 1vh;
  align-items: start;
`;

export const StyledHomeContent = styled.div<StyledHomeContainerProps>`
  flex-grow: 1;
  border-radius: 25px 0 0 0;
  padding: 25px;
  background-color: ${(p) => p.bg ?? primaryColor};
  box-shadow:
    inset 0 1px 2px #ffffff30,
    0 1px 2px #00000030,
    0 2px 4px #00000015;
`;

interface StyledActionProps {
  bg?: string;
  color?: string;
  roundness?: number;
}

export const StyledAction = styled(Link)<StyledActionProps>`
  align-content: center;
  text-align: center;
  border-radius: ${(p) => (p.roundness ? p.roundness * 2 : 1)}px;
  width: 90%;
  font-size: 20px;
  padding: 5px;
  background-color: ${(p) => p.bg ?? primaryColor};
  color: ${(p) => p.color ?? primaryColor};
  margin: 40px 0;

  &:link {
    text-decoration: none;
  }
`;
