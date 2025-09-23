import { CanvasWidget } from "@projectstorm/react-canvas-core";
import styled from "styled-components";

export const StyledLibraryCanvas = styled(CanvasWidget)`
  height: 200px;
  margin: 10px;
`;

export const StyledLibraryEntry = styled.div`
  display: flex;
  flex-direction: column;
  width: 90%;
  background: inherit;
`;