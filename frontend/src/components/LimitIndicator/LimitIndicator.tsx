import React from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledIndicatorContainer,
  StyledLimitIndicator,
  StyledLimitIndicatorContainer,
  StyledLimitIndicatorText,
} from "BtStyles/LimitIndicator/LimitIndicator.styles";

function convertBytes(bytes: number): string[] {
  const decimals = 2;

  if (decimals < 0) {
    throw new Error(`Invalid decimals ${decimals}`);
  }

  const base = 1000;
  const units = ["Bytes", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"];

  const i = Math.floor(Math.log(bytes) / Math.log(base));

  return [`${(bytes / Math.pow(base, i)).toFixed(decimals)}`, `${units[i]}`];
}

const Indicator = ({
  size,
  max,
  units,
}: {
  size: number;
  max: number;
  units: string;
}) => {
  const theme = useBtTheme();
  const perc = (size * 100) / max;

  let unit_size = units;
  let unit_max = units;
  let display_size = `${size}`;
  let display_max = `${max}`;

  if (unit_size === "size") {
    const size_data = convertBytes(size);
    unit_size = size_data[1];
    display_size = size_data[0];
    const max_data = convertBytes(max);
    unit_max = max_data[1];
    display_max = max_data[0];
  }

  if (max < 0) {
    return (
      <StyledIndicatorContainer>
        <StyledLimitIndicatorContainer bg={theme.palette.bgLight}>
          <StyledLimitIndicator bg={theme.palette.primary} width={10} />
        </StyledLimitIndicatorContainer>
        <StyledLimitIndicatorText
          color={theme.palette.text}
          full_bg={theme.palette.error}
          width={0}
        >
          <label id="indic">{`No ${units} limit`}</label>
        </StyledLimitIndicatorText>
      </StyledIndicatorContainer>
    );
  }

  return (
    <StyledIndicatorContainer>
      <StyledLimitIndicatorContainer bg={theme.palette.bgLight}>
        {perc > 80 ? (
          <StyledLimitIndicator bg={theme.palette.error} width={perc} />
        ) : (
          <StyledLimitIndicator bg={theme.palette.primary} width={perc} />
        )}
      </StyledLimitIndicatorContainer>
      <StyledLimitIndicatorText
        color={theme.palette.text}
        full_bg={theme.palette.error}
        width={perc}
      >
        <label id="indic">{`${display_size} ${unit_size}`}</label>
        <label id="base">{` / ${display_max} ${unit_max}`}</label>
      </StyledLimitIndicatorText>
    </StyledIndicatorContainer>
  );
};

export const Skeleton = () => {
  const theme = useBtTheme();

  return (
    <StyledIndicatorContainer>
      <StyledLimitIndicatorContainer bg={theme.palette.bgLight}>
        <StyledLimitIndicator bg={theme.palette.primary} width={0} />
      </StyledLimitIndicatorContainer>
      <StyledLimitIndicatorText
        color={theme.palette.text}
        full_bg={theme.palette.error}
        width={0}
      >
        <label id="indic">{`...`}</label>
        <label id="base">{` / ...`}</label>
      </StyledLimitIndicatorText>
    </StyledIndicatorContainer>
  );
};

export default Indicator;
