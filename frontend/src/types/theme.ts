import { Theme } from "jderobot-ide-interface";

export interface BtTheme extends Theme {
  switch: (themeType: string) => void;
  btEditor: {
    border: string;
    shadow: string;
    running: string;
    success: string;
    failure: string;
    invalid: string;
    roundness?: number;
  },
  hoverStyle: string;
}
