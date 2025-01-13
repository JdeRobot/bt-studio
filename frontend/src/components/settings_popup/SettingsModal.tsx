import { useContext } from "react";
import "react-color-palette/css";
import "./SettingsModal.css";
import Modal from "../Modal/Modal";
import { ReactComponent as CloseIcon } from "../Modal/img/close.svg";

import Section from "./sections/Section";
import SubSection from "./sections/SubSection";
import Setting from "./sections/Setting";

import Dropdown from "./options/Dropdown";

import { OptionsContext, SettingsData } from "../options/Options";
import { useError } from "../error_popup/ErrorModal";

import { saveProjectConfig } from "../../api_helper/TreeWrapper";

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
  // const [color, setColor] = useColor("rgb(128 0 128)");
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
        hasCloseBtn={true}
        isOpen={isOpen}
        onClose={onClose}
      >
        <form
          onSubmit={onSubmit}
          onReset={() => {
            handleCancel(settings);
          }}
          style={{ display: "flex", flexDirection: "column", flexGrow: "1" }}
        >
          <div className="bt-modal-titlebar">
            <label
              className="bt-modal-titlebar-title"
              htmlFor="actionName"
              style={{ textAlign: "center" }}
            >
              Settings
            </label>
            <CloseIcon
              className="bt-modal-titlebar-close bt-icon"
              onClick={() => {
                handleCancel(settings);
              }}
              fill={"var(--icon)"}
            />
          </div>
          <div
            className="bt-form-row"
            style={{ display: "flex", flexDirection: "column", flexGrow: "1" }}
          >
            <ul className="bt-settings-entry-list">
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
                <SubSection title="Color theme">
                  <Setting title="Set color theme">
                    <Dropdown
                      setting={settings.theme}
                      possibleValues={["dark", "light"]}
                    />
                  </Setting>
                </SubSection>
                {/* <SubSection title="Editor">
                  <Setting title="Show actions accent color">
                    <Checkbox setting={settings.editorShowAccentColors} />
                  </Setting>
                </SubSection> */}
              </Section>
              <Section title="Behaviour Tree">
                <SubSection title="Execution settings">
                  <Setting title="Order of execution of the behavior tree">
                    {/* Add explanation here */}
                    <Dropdown
                      setting={settings.btOrder}
                      possibleValues={["bottom-to-top", "top-to-bottom"]}
                    />
                  </Setting>
                </SubSection>
              </Section>
            </ul>
          </div>
        </form>
      </Modal>
    );
  }
};

export default SettingsModal;
