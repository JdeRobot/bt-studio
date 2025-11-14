import styled from "styled-components";

const primaryColor = "#666";

interface StyledAppContainerProps {
  bg?: string;
}

export const StyledAppContainer = styled.div<StyledAppContainerProps>`
  display: flex;
  flex-direction: column;
  min-height: 100vh;
  background-color: ${(p) => p.bg ?? primaryColor};

  &h1 {
    font-size: 25px;
  }

  &h2 {
    font-size: 18px;
  }

  &h3 {
    font-size: 12px;
  }
`;

