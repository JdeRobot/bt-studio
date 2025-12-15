import React, { useContext } from "react";
import { createContext, useState } from "react";

const ProjectSettingsContext = createContext<SettingsData>({
  editorShowAccentColors: {
    setter: () => {},
    value: false,
    default_value: false,
  },
  btOrder: {
    setter: () => {},
    value: "bottom-to-top",
    default_value: "bottom-to-top",
  },
});

export interface SettingData<Type> {
  setter: Function;
  value: Type;
  default_value: Type;
}

export interface SettingsData {
  editorShowAccentColors: SettingData<boolean>;
  btOrder: SettingData<string>;
}

const useProjectSettings = () => useContext(ProjectSettingsContext);

const ProjectSettingsProvider = ({ children }: { children: any }) => {
  // TODO: try to not repeat the default values
  const [editorShowAccentColors, setEditorShowAccentColors] = useState(false);
  const [btOrder, setBtOrder] = useState("bottom-to-top");

  // Setting => name: {setter: function, value: name, default_value: default_value}
  const settings: SettingsData = {
    editorShowAccentColors: {
      setter: setEditorShowAccentColors,
      value: editorShowAccentColors,
      default_value: false,
    },
    btOrder: {
      setter: setBtOrder,
      value: btOrder,
      default_value: "bottom-to-top",
    },
  };

  return (
    <ProjectSettingsContext.Provider value={settings}>
      {children}
    </ProjectSettingsContext.Provider>
  );
};

export { useProjectSettings, ProjectSettingsProvider };
