import { contrastSelector } from "jderobot-ide-interface";
import styled from "styled-components";

const primaryColor = "#666";

function rgbToHex(rgb: string): string {
  // Set seperator based on the rgb string provided
  // checks for both rgb and rgba string.
  const seperator = rgb.indexOf(",") > -1 ? "," : " ";

  // Array containing hex values converted from the rgbArray
  const hexArray: string[] = [];

  // Check if it is rgba
  const isRgba = rgb.indexOf("rgba") > -1;

  // Remove slash from string if present
  // make sure to replace the space before it as well or we will have an
  // empty string inside the rgbArray
  rgb = rgb.replace(" /", "");

  // Convert the color string to an array
  const rgbArray = rgb
    .substring(isRgba ? 5 : 4)
    .split(")")[0]
    .split(seperator);

  // Convert rgb values from rgbArray to hex values and add it to the hexArray
  rgbArray.forEach((colorValue, index) => {
    const trimmedColorValue = colorValue.trim();
    let hexValue = trimmedColorValue;

    // Convert the percentage value to proper rgb value
    // so 100% becomes 255 => (100 / 100) * 255
    if (trimmedColorValue.indexOf("%") > -1) {
      hexValue = String(
        Math.round(
          (+trimmedColorValue.substring(0, trimmedColorValue.length - 1) /
            100) *
            255
        )
      );
    }

    // If the index is 3, this is the alpha value,
    // check if it is less than or equal to 1 becuase,
    // if percentage format is used then rather than being 0.9 it will be 90% and
    // we don't want to do any rounding as it is already rounded.
    // Then multiply it with 255 and then round the value
    // So, alpha value of 0.4 becomes -> 102
    if (index == 3 && +hexValue <= 1) {
      hexValue = String(Math.round(+hexValue * 255));
    }

    // Conver hexValue to a number and then to a hex string
    hexValue = (+hexValue).toString(16);

    // If only one hex value is present, then add a leading 0
    if (hexValue.length == 1) hexValue = "0" + hexValue;

    // Push the hex values to the hexArray
    hexArray.push(hexValue);
  });

  // Return the Hex string
  return "#" + hexArray.join("");
}

interface StyledBTModelContainerProps {
  bg: string;
  borderColor?: string;
  roundness?: number;
  lightText?: string;
  darkText?: string;
}

export const StyledBTModelContainer = styled.div<StyledBTModelContainerProps>`
  background-color: ${(p) => p.bg};
  border: 2px solid ${(p) => p.borderColor ?? primaryColor};
  border-radius: ${(p) => p.roundness ?? 1}px;
  padding: 10px;
  width: 100%;

  & label, button {
    color: ${(p) => contrastSelector(p.lightText, p.darkText, rgbToHex(p.bg)) ?? primaryColor} !important;
  }
`;

export const StyledBTModelName = styled.label`
  width: 100%;
  display: block;
  border: none;
  font-size: large;
  font-weight: bold;
  text-align: center;
  margin-bottom: 10px;
  margin-left: 5px;
`;

export const StyledBTModelIO = styled.div`
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-gap: 10px;
`;

interface StyledBTModelIOEntryProps {
  bg: string;
  roundness?: number;
  type: "input" | "output";
}


const handleEntryType = (p: StyledBTModelIOEntryProps) => {
  switch (p.type) {
    case "input":
      return `grid-template-columns: auto 20px;`;
    case "output":
      return `grid-template-columns: 20px auto;`;
  }
};

export const StyledBTModelIOEntry = styled.div<StyledBTModelIOEntryProps>`
  border-radius: ${(p) => p.roundness ?? 1}px;
  padding: 5px;
  display: grid;
  grid-gap: 5px;
  height: 2em;
  ${handleEntryType}

  &:hover {
    background-color: ${(p) => p.bg};
    filter: var(--hover-strong);
    box-shadow: var(--shadow);
    & button {
      visibility: visible;
    }
  }

  & label {
    text-align: left !important;
    font-size: medium !important;
    display: block;
    overflow-x: scroll;
    -ms-overflow-style: none; /* IE and Edge */
    scrollbar-width: none; /* Firefox */

    &::-webkit-scrollbar {
      display: none;
    }
  }
`;

interface StyledBTModelIODeleteProps {
  roundness?: number;
  bg: string;
  visible?: boolean;
}

export const StyledBTModelIODelete = styled.button<StyledBTModelIODeleteProps>`
  padding: 0px;
  border-radius: ${(p) => p.roundness ?? 1}px !important;
  border: none;
  background-color: ${(p) => p.bg} !important;
  height: unset !important;
  visibility: ${(p) => p.visible ? "visible" : "hidden"};
  display: flex;
  justify-content: center;
  align-items: center;
  box-shadow: var(--shadow);
`;

interface RoundnessProps {
  roundness?: number;
}

export const StyledBTModelInputContainer = styled.div<RoundnessProps>`
  border-radius: ${(p) => p.roundness ?? 1}px !important;
  padding: 5px;
  display: grid;
  grid-template-columns: auto 20px 20px;
  grid-gap: 5px;
  height: 2em;
`;

interface StyledBTModelInputProps {
  roundness?: number;
  bg: string;
  color?: string;
}

export const StyledBTModelInput = styled.input<StyledBTModelInputProps>`
  border-radius: ${(p) => p.roundness ?? 1}px !important;
  border: none;
  padding-left: 5px;
  width: 100%;
  font-size: medium;
  background-color: ${(p) => p.bg} !important;
  color: ${(p) => p.color};

  &:focus {
    outline: none;
  }
`;

interface StyledBTModelAddProps {
  roundness?: number;
  bg: string;
}

export const StyledBTModelAdd = styled.button<StyledBTModelAddProps>`
  background: none !important;
  border-radius: ${(p) => p.roundness ?? 1}px !important;
  border: none;
  width: 100%;
  font-size: larger;

  &:focus {
    outline: none;
    box-shadow: none;
  }

  &:hover {
    background-color: ${(p) => p.bg}!important;
    filter: var(--hover-strong);
    box-shadow: var(--shadow);
  }
`;