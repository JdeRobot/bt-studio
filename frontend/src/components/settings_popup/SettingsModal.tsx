import { useContext } from "react";
import "react-color-palette/css";
import "./SettingsModal.css";

import Section from "./sections/Section";
import SubSection from "./sections/SubSection";
import Setting from "./sections/Setting";

import { OptionsContext, SettingsData } from "../options/Options";
import {
  useError,
  Modal,
  ModalTitlebar,
  ModalRow,
} from "jderobot-ide-interface";

import { saveProjectConfig } from "../../api_helper/TreeWrapper";
import Checkbox from "./options/Checkbox";
import { StyledSettingsListConatiner } from "Styles/Modal/Settings/Settings.styles";
import { useBtTheme } from "Contexts/BtThemeContext";

const SettingsModal = ({
  onSubmit,
  isOpen,
  onClose,
  currentProjectname,
}: {
  onSubmit: (data: unknown) => void;
  isOpen: boolean;
  onClose: Function;
  currentProjectname: string;
}) => {
  const theme = useBtTheme();
  const settings = useContext(OptionsContext);
  const { error } = useError();

  // useEffect(() => {
  //   console.log("rgb("+Math.round(color.rgb.r)+","+Math.round(color.rgb.g)+","+Math.round(color.rgb.b)+")")
  //   document.documentElement.style.setProperty("--header", "rgb("+Math.round(color.rgb.r)+","+Math.round(color.rgb.g)+","+Math.round(color.rgb.b)+")");
  // }, [color]);

  const handleCancel = async (settings: SettingsData) => {
    // Save settings
    let json_settings: { name: string; config: { [id: string]: any } } = {
      name: currentProjectname,
      config: {},
    };

    Object.entries(settings).map(([key, setting]) => {
      json_settings.config[key] = setting.value;
    });

    try {
      await saveProjectConfig(
        currentProjectname,
        JSON.stringify(json_settings),
      );
    } catch (e) {
      console.error("Error saving config:", e);
      error("Error saving config: " + error);
    }
    onClose();
  };

  if (isOpen) {
    return (
      <Modal
        id="settings-modal"
        isOpen={isOpen}
        onClose={onClose}
        onSubmit={onSubmit}
        onReset={() => {
          handleCancel(settings);
        }}
      >
        <ModalTitlebar
          title="Settings"
          htmlFor="actionName"
          hasClose
          handleClose={() => {
            handleCancel(settings);
          }}
        />
        <ModalRow>
          <StyledSettingsListConatiner scrollbar={theme.palette.scrollbar}>
            {/* <Section title="General">
                <SubSection title="Accent Colors">
                  <Setting title ="Turn on project accent color">
                  </Setting>
                  <Setting title ="Project accent color">
                    <Saturation height={100} color={color} onChange={setColor} />
                    <Hue width={300} color={color} onChange={setColor} />
                  </Setting>
                </SubSection>
              </Section> */}
            <Section title="Appearance">
              <SubSection title="Editor">
                <Setting title="Show actions accent color">
                  <Checkbox setting={settings.editorShowAccentColors} />
                </Setting>
              </SubSection>
            </Section>
          </StyledSettingsListConatiner>
        </ModalRow>
      </Modal>
    );
  }
};

export default SettingsModal;
