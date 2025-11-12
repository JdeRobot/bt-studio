import { CanvasWidget } from "@projectstorm/react-canvas-core";
import styled from "styled-components";

const primaryColor = "#666";

interface StyledBTCanvasProps {
  bgColor?: string;
}

export const StyledBTCanvas = styled(CanvasWidget)<StyledBTCanvasProps>`
  height: 100%;
  width: 100%;
  background-color: ${(p) => p.bgColor ?? primaryColor};
`;