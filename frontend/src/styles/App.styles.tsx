import styled from "styled-components";

const primaryColor = "#666";

interface StyledAppContainerProps {
  bg?: string;
  hoverStyle?: string;
}

const handleHover = (p: StyledAppContainerProps) => {
  switch (p.hoverStyle) {
    case "lighten":
      return `--hover-light: brightness(1.25);--hover-strong: brightness(1.5);`;
    case "darken":
      return `--hover-light: brightness(0.95);--hover-strong: brightness(0.85);`;
  }
};

export const StyledAppContainer = styled.div<StyledAppContainerProps>`
  display: flex;
  flex-direction: column;
  min-height: 100vh;
  background-color: ${(p) => p.bg ?? primaryColor};

  ${handleHover}

  --shadow: inset 0 1px 2px #ffffff30, 0 1px 2px #00000030, 0 2px 4px #00000015;

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
