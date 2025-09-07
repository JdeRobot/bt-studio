import { Theme } from "jderobot-ide-interface";

export interface BtTheme extends Theme {
  switch: (themeType: string) => void;
}
