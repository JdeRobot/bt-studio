import { createContext, useState } from "react";

const OptionsContext = createContext<SettingsData>({
  editorShowAccentColors: {
    setter: () => {},
    value: true,
    default_value: true,
  },
  theme: { setter: () => {}, value: "dark", default_value: "dark" },
  btOrder: {
    setter: () => {},
    value: "bottom-to-top",
    default_value: "bottom-to-top",
  },
});

export interface SettingsData {
  editorShowAccentColors: {
    setter: Function;
    value: boolean;
    default_value: boolean;
  };
  theme: { setter: Function; value: string; default_value: string };
  btOrder: {
    setter: Function;
    value: string;
    default_value: string;
  };
}

const OptionsProvider = ({ children }: { children: any }) => {
  // TODO: try to not repeat the default values
  const [editorShowAccentColors, setEditorShowAccentColors] = useState(true);
  const [theme, setTheme] = useState("dark");
  const [btOrder, setBtOrder] = useState("bottom-to-top");

  // Setting => name: {setter: function, value: name, default_value: default_value}
  const settings: SettingsData = {
    editorShowAccentColors: {
      setter: setEditorShowAccentColors,
      value: editorShowAccentColors,
      default_value: true,
    },
    theme: { setter: setTheme, value: theme, default_value: "dark" },
    btOrder: {
      setter: setBtOrder,
      value: btOrder,
      default_value: "bottom-to-top",
    },
  };

  return (
    <OptionsContext.Provider value={settings}>
      {children}
    </OptionsContext.Provider>
  );
};

export { OptionsContext, OptionsProvider };
