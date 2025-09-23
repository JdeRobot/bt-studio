import { PortWidget } from "@projectstorm/react-diagrams";
import { strRGBToLuminance } from "Components/helper/colorHelper";
import styled from "styled-components";
import { BTStatus } from "Types/index";

const primaryColor = "#666";

interface StyledNodeContainerProps {
  borderColor?: string;
  shadowColor?: string;
  roundness?: number;
  color: string;
  selected: boolean;
  status?: BTStatus;
  statusRunning: string;
  statusSuccess: string;
  statusFailure: string;
  statusInvalid: string;
}

const handleContrast = (p: StyledNodeContainerProps) => {
  const showLightText = strRGBToLuminance(p.color) <= 0.5;

  if (showLightText) {
    return `color: #fff;`;
  } else {
    return `color: #000;`;
  }
};

const handleSelected = (p: StyledNodeContainerProps) => {
  if (p.selected) {
    return `box-shadow: 0 0 12px ${p.shadowColor ?? primaryColor};`;
  }
};

const handleState = (p: StyledNodeContainerProps) => {
  switch (p.status) {
    case "RUNNING":
      return `background-color: ${p.statusRunning};`;
    case "SUCCESS":
      return `background-color: ${p.statusSuccess};`;
    case "FAILURE":
      return `background-color: ${p.statusFailure};`;
    case "INVALID":
      return `background-color: ${p.statusInvalid};`;
    default:
      break;
  }
};

export const StyledNodeContainer = styled.div<StyledNodeContainerProps>`
  display: flex;
  justify-content: space-between;
  border: 2px solid ${(p) => p.borderColor ?? primaryColor};
  border-radius: ${(p) => p.roundness ?? 1}px;
  padding: ${(p) => p.roundness ?? 1}px;
  flex-direction: column;
  cursor: pointer;
  background-color: ${(p) => p.color};
  ${handleContrast}
  ${handleSelected}
  ${handleState}
`;

export const StyledNodeSection = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
`;

interface StyledNodeDiagramPortsProps {
  type: "parent" | "children";
}

const handleType = (p: StyledNodeDiagramPortsProps) => {
  switch (p.type) {
    case "parent":
      return `margin-right: 5px; margin-left: -15px;`;
    case "children":
      return `margin-right: -15px; margin-left: 5px;`;
  }
};

export const StyledNodeDiagramPorts = styled.div<StyledNodeDiagramPortsProps>`
  ${handleType}
`;

interface StyledNodePortProps {
  color?: string;
}

export const StyledNodePort = styled.div<StyledNodePortProps>`
  width: 10px;
  height: 10px;
  border-radius: 100%;
  background-color: ${(p) => p.color};
`;

interface StyledNodeTagPortsProps {
  type: "input" | "output";
}

const handleTagType = (p: StyledNodeTagPortsProps) => {
  switch (p.type) {
    case "input":
      return `margin-left: -10px;`;
    case "output":
      return `align-items: flex-end; margin-right: -10px;`;
  }
};

export const StyledNodeTagPorts = styled.div<StyledNodeTagPortsProps>`
  display: flex;
  flex-direction: column;
  ${handleTagType}
`;

export const StyledNodeTagPort = styled(PortWidget)<StyledNodePortProps>`
  width: 10px;
  height: 10px;
  background-color: ${(p) => p.color};
`;

export const StyledNodeTitle = styled.div`
  font-weight: bold;
`;

const handleTagPortContainerType = (p: StyledNodeTagPortsProps) => {
  switch (p.type) {
    case "input":
      return `margin-left: -4px; margin-right: 5px;`;
    case "output":
      return `margin-left: 4px; margin-right: -5px;`;
  }
};

export const StyledNodeTagPortContainer = styled.div<StyledNodeTagPortsProps>`
  display: inline-flex;
  align-items: center;
  margin-top: 5px;
  ${handleTagPortContainerType}
`;

const handleTagLabelType = (p: StyledNodeTagPortsProps) => {
  switch (p.type) {
    case "input":
      return `padding-left: 5px;`;
    case "output":
      return `padding-right: 5px;`;
  }
};

export const StyledNodeTagPortLabel= styled.div<StyledNodeTagPortsProps>`
  display: inline-block;
  vertical-align: middle;
  ${handleTagLabelType}
`;

interface StyledTagContainerProps {
  borderColor?: string;
  shadowColor?: string;
  roundness?: number;
  color: string;
  selected: boolean;
  type: "input" | "output";
}

const handleTagContrast = (p: StyledTagContainerProps) => {
  const showLightText = strRGBToLuminance(p.color) <= 0.5;

  if (showLightText) {
    return `color: #fff;`;
  } else {
    return `color: #000;`;
  }
};

const handleTagSelected = (p: StyledTagContainerProps) => {
  if (p.selected) {
    return `box-shadow: 0 0 12px ${p.shadowColor ?? primaryColor};`;
  }
};

const handleTagContainerType = (p: StyledTagContainerProps) => {
  switch (p.type) {
    case "input":
      return `padding-right: 5px;`;
    case "output":
      return `padding-left: 5px;`;
  }
};

export const StyledTagContainer = styled.div<StyledTagContainerProps>`
  display: flex;
  justify-content: space-between;
  border: 2px solid ${(p) => p.borderColor ?? primaryColor};
  border-radius: ${(p) => p.roundness ?? 1}px;
  padding-top: 5px;
  padding-bottom: 5px;
  flex-direction: column;
  cursor: pointer;
  background-color: ${(p) => p.color};
  ${handleTagContrast}
  ${handleTagSelected}
  ${handleTagContainerType}
`;