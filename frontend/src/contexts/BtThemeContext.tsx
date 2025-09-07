import { ThemeProvider } from "jderobot-ide-interface";
import { createContext, ReactNode, useContext, useState } from "react";
import { BtTheme } from "../types";

interface BtThemeProviderProps {
  children?: ReactNode;
}

const darkTheme: BtTheme = {
  switch: (themeType: string) => {},
  palette: {
    text: "#ededf2",
    darkText: "#000000",
    placeholderText: "#a6a6bf",
    success: "#29ac29",
    warning: "#f9e86d",
    error: "#802626",
    background: "#16161d",
    primary: "#134f53",
    secondary: "#1d777c",
    scrollbar: "#6f6f90",
    border: {
      warning: "#ffe100",
      error: "#772222",
      info: "#134f53",
    },
    progressBar: {
      background: "#134f53",
      color: "#1d777c",
    },
    button: {
      error: "#9e2e2e",
      success: "#29ac29",
      warning: "#ffe100",
      info: "#134f53",
      hoverError: "#c63939",
      hoverSuccess: "#29ac29",
      hoverWarning: "#ccb400",
      hoverInfo: "#1d777c",
    },
    selectedGradient:
      "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
  },
  roundness: 5,
  transitionSpeed: 200,
  monacoTheme: "dark",
};

const lightTheme: BtTheme = {
  switch: (themeType: string) => {},
  palette: {
    text: "#ededf2",
    darkText: "#000000",
    placeholderText: "#a6a6bf",
    success: "#29ac29",
    warning: "#f9e86d",
    error: "#802626",
    background: "#ffffffff",
    primary: "#134f53",
    secondary: "#1d777c",
    scrollbar: "#6f6f90",
    border: {
      warning: "#ffe100",
      error: "#772222",
      info: "#134f53",
    },
    progressBar: {
      background: "#134f53",
      color: "#1d777c",
    },
    button: {
      error: "#9e2e2e",
      success: "#29ac29",
      warning: "#ffe100",
      info: "#134f53",
      hoverError: "#c63939",
      hoverSuccess: "#29ac29",
      hoverWarning: "#ccb400",
      hoverInfo: "#1d777c",
    },
    selectedGradient:
      "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
  },
  roundness: 5,
  transitionSpeed: 200,
  monacoTheme: "light",
};

const BtThemeContext = createContext(darkTheme);
export const useBtTheme = () => useContext(BtThemeContext) ?? darkTheme;

export const BtThemeProvider = ({ children }: BtThemeProviderProps) => {
  const [currentTheme, setCurrentTheme] = useState<BtTheme>(
    window.localStorage.getItem("themeType") !== null
      ? window.localStorage.getItem("themeType") === "light"
        ? lightTheme
        : darkTheme
      : darkTheme,
  );

  const themeSwitchHandler = (themeType: string) => {
    window.localStorage.setItem("themeType", themeType);
    switch (themeType) {
      case "light":
        setCurrentTheme(lightTheme);
        break;
      case "dark":
        setCurrentTheme(darkTheme);
        break;
      default:
        break;
    }
  };

  return (
    <BtThemeContext.Provider
      value={{ ...currentTheme, switch: themeSwitchHandler }}
    >
      <ThemeProvider theme={currentTheme}>{children}</ThemeProvider>
    </BtThemeContext.Provider>
  );
};
