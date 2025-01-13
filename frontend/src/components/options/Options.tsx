import { createContext, useState } from "react";

const OptionsContext = createContext<SettingsData>({
  editorShowAccentColors: {
    setter: () => {},
    value: false,
    default_value: false,
  },
  theme: { setter: () => {}, value: "dark", default_value: "dark" },
  btOrder: {
    setter: () => {},
    value: "bottom-to-top",
    default_value: "bottom-to-top",
  },
});

export interface SettingData<Type>{
  setter: Function;
  value: Type;
  default_value: Type;
}

export interface SettingsData {
  editorShowAccentColors: SettingData<boolean>;
  theme: SettingData<string>;
  btOrder: SettingData<string>;
}

const OptionsProvider = ({ children }: { children: any }) => {
  // TODO: try to not repeat the default values
  const [editorShowAccentColors, setEditorShowAccentColors] = useState(false);
  const [theme, setTheme] = useState("dark");
  const [btOrder, setBtOrder] = useState("bottom-to-top");

  // Setting => name: {setter: function, value: name, default_value: default_value}
  const settings: SettingsData = {
    editorShowAccentColors: {
      setter: setEditorShowAccentColors,
      value: editorShowAccentColors,
      default_value: false,
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
