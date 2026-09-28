import React, { useState, useEffect, useRef } from "react";
import {
  createCombinedUniverse,
  listDockerWorlds,
  listDockerRobots,
  captureRobotPose,
} from "BtApi/TreeWrapper";
import {
  ModalInputBox,
  ModalInputDropdown,
  ModalRow,
  ModalTitlebar,
  useError,
} from "jderobot-ide-interface";

interface Option {
  id: string;
  name: string;
  type?: string;
}

const initialFormData = {
  universeName: "",
  worldName: "",
  robotName: "None",
  useCustomPose: false,
  x: "0",
  y: "0",
  z: "0",
  yaw: "0",
};

const CreateCombinedPage = ({
  setVisible,
  visible,
  onClose,
  currentProject,
}: {
  setVisible: Function;
  visible: boolean;
  onClose: Function;
  currentProject: string;
}) => {
  const { error } = useError();
  const [capturing, setCapturing] = useState(false);

  const handleCapturePose = async () => {
    setCapturing(true);
    try {
      const pose = await captureRobotPose();
      setFormState(prev => ({
        ...prev,
        x: String(pose[0]),
        y: String(pose[1]),
        z: String(pose[2]),
        yaw: String(pose[5])
      }));
    } catch (e: any) {
      error(e.response?.data?.message || e.message || "Failed to capture pose from simulator. Make sure the simulation is running.");
    } finally {
      setCapturing(false);
    }
  };

  const focusInputRef = useRef<any>(null);
  const worldDropdownRef = useRef<any>(null);
  const robotDropdownRef = useRef<any>(null);

  const [formState, setFormState] = useState(initialFormData);
  const [worlds, setWorlds] = useState<Option[]>([]);
  const [robots, setRobots] = useState<Option[]>([]);

  const [openWorldDropdown, setOpenWorldDropdown] = useState<boolean>(false);
  const [openRobotDropdown, setOpenRobotDropdown] = useState<boolean>(false);

  const loadOptions = async () => {
    try {
      const worldsList = await listDockerWorlds();
      const robotsList = await listDockerRobots();
      setWorlds(worldsList);
      setRobots(robotsList);
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching worlds/robots list: " + e.message);
        error("Error while fetching worlds/robots list: " + e.message);
      }
    }
  };

  useEffect(() => {
    if (visible && focusInputRef.current) {
      setTimeout(() => {
        focusInputRef.current.focus();
      }, 0);
    }
    loadOptions();
  }, [visible]);

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    setFormState((prevFormData) => {
      const updated = {
        ...prevFormData,
        [name]: value,
      };
      if (name === "worldName") {
        const selectedWorld = worlds.find((w) => w.name === value);
        if (selectedWorld && selectedWorld.type !== "gz") {
          updated.robotName = "None";
        }
      }
      return updated;
    });
  };

  const handleCancel = () => {
    if (currentProject !== "") {
      onClose();
    }
  };

  const handleCreate = async () => {
    if (formState.universeName === "" || formState.worldName === "") {
      return;
    }

    const selectedWorld = worlds.find((w) => w.name === formState.worldName);
    if (!selectedWorld) {
      error("Invalid world selected");
      return;
    }

    let robotId = "None";
    if (formState.robotName !== "None" && formState.robotName !== "") {
      const selectedRobot = robots.find((r) => r.name === formState.robotName);
      if (!selectedRobot) {
        error("Invalid robot selected");
        return;
      }
      robotId = selectedRobot.id;
    }

    let customPose = null;
    if (formState.useCustomPose) {
      const x = parseFloat(formState.x);
      const y = parseFloat(formState.y);
      const z = parseFloat(formState.z);
      const yaw = parseFloat(formState.yaw);

      if (isNaN(x) || isNaN(y) || isNaN(z) || isNaN(yaw)) {
        error("Please enter valid numbers for the coordinates.");
        return;
      }

      if (x < -200 || x > 200 || y < -200 || y > 200) {
        error("Coordinates X and Y must be between -200 and 200 meters to keep the robot inside the world bounds.");
        return;
      }

      if (z < -1 || z > 100) {
        error("Coordinate Z must be between -1 and 100 meters.");
        return;
      }

      if (yaw < -3.15 || yaw > 3.15) {
        error("Yaw must be in radians, between -3.14 and 3.14 (approximately -180 to 180 degrees).");
        return;
      }

      customPose = [x, y, z, 0.0, 0.0, yaw];
    }

    try {
      await createCombinedUniverse(
        currentProject,
        formState.universeName,
        selectedWorld.id,
        robotId,
        customPose
      );
      setVisible(false);
    } catch (e) {
      if (e instanceof Error) {
        error("Failed to create combined universe: " + e.message);
      }
    }
  };

  const closeDropdowns = (e: any) => {
    if (openWorldDropdown && !worldDropdownRef.current?.contains(e.target)) {
      setOpenWorldDropdown(false);
    }
    if (openRobotDropdown && !robotDropdownRef.current?.contains(e.target)) {
      setOpenRobotDropdown(false);
    }
  };

  useEffect(() => {
    document.addEventListener("mousedown", closeDropdowns);
    return () => {
      document.removeEventListener("mousedown", closeDropdowns);
    };
  }, [openWorldDropdown, openRobotDropdown]);

  const worldNames = worlds
    .filter((w) => w.type === "gz")
    .map((w) => w.name);
  const robotNames = ["None", ...robots.map((r) => r.name)];

  const selectedWorld = worlds.find((w) => w.name === formState.worldName);
  const isIgnitionGazebo = selectedWorld && selectedWorld.type === "gz";

  return (
    <>
      <ModalTitlebar
        title="Create Combined World"
        htmlFor="actionName"
        hasClose
        hasBack
        handleClose={() => {
          handleCancel();
        }}
        handleBack={() => {
          setVisible(false);
        }}
      />
      <ModalRow type="input">
        <ModalInputBox
          isInputValid={true}
          ref={focusInputRef}
          id="universeName"
          placeholder="World Name"
          onChange={handleInputChange}
          description="A unique name that is used for the world folder and other
            resources. The name should be in lower case without spaces."
          type="text"
          autoComplete="off"
          required
          maxLength={20}
        />
      </ModalRow>
      <ModalRow type="input">
        <ModalInputDropdown
          isInputValid={true}
          ref={worldDropdownRef}
          entries={worldNames}
          id="worldName"
          placeholder="Select Scene"
          onChange={handleInputChange}
          type="text"
          required
        ></ModalInputDropdown>
      </ModalRow>
      {isIgnitionGazebo ? (
        <ModalRow type="input">
          <ModalInputDropdown
            isInputValid={true}
            ref={robotDropdownRef}
            entries={robotNames}
            id="robotName"
            placeholder="Select Robot"
            onChange={handleInputChange}
            type="text"
            required
          ></ModalInputDropdown>
        </ModalRow>
      ) : (
        formState.worldName !== "" && (
          <div style={{
            padding: "10px 16px",
            color: "#6c757d",
            fontSize: "0.9em",
            fontStyle: "italic",
            backgroundColor: "#202020",
            borderRadius: "4px",
            margin: "0 16px 12px 16px",
            borderLeft: "4px solid #1976d2"
          }}>
            This classic/drone scene includes its own default robot. Custom robot selection is locked to 'None'.
          </div>
        )
      )}
      {formState.worldName !== "" && (
        <>
          <div style={{ padding: "0 16px 12px 16px", display: "flex", alignItems: "center" }}>
            <input
              type="checkbox"
              id="useCustomPose"
              name="useCustomPose"
              checked={formState.useCustomPose}
              onChange={(e) => {
                setFormState(prev => ({ ...prev, useCustomPose: e.target.checked }));
              }}
              style={{ marginRight: "8px", cursor: "pointer", width: "16px", height: "16px" }}
            />
            <label htmlFor="useCustomPose" style={{ color: "#E0E0E0", cursor: "pointer", fontSize: "0.9em", userSelect: "none" }}>
              Customize starting pose (x, y, z, yaw)
            </label>
          </div>
          {formState.useCustomPose && (
            <div style={{
              margin: "0 16px 12px 16px",
              padding: "16px",
              backgroundColor: "#181818",
              border: "1px solid #333",
              borderRadius: "4px"
            }}>
              <div style={{ color: "#a5a5a5", fontSize: "0.8em", marginBottom: "12px", fontStyle: "italic" }}>
                ⚠️ Warning: Coordinates X/Y must be between -200m and 200m. Z must be between -1m and 100m.
              </div>
              <div style={{ display: "flex", gap: "12px", marginBottom: "12px" }}>
                <div style={{ flex: 1 }}>
                  <label style={{ fontSize: "0.8em", color: "#8c8c8c", display: "block", marginBottom: "4px" }}>X (meters)</label>
                  <input
                    type="number"
                    name="x"
                    value={formState.x}
                    onChange={handleInputChange}
                    min="-200"
                    max="200"
                    step="0.1"
                    style={{
                      width: "100%",
                      padding: "8px",
                      backgroundColor: "#202020",
                      border: "1px solid #444",
                      color: "#fff",
                      borderRadius: "4px"
                    }}
                  />
                </div>
                <div style={{ flex: 1 }}>
                  <label style={{ fontSize: "0.8em", color: "#8c8c8c", display: "block", marginBottom: "4px" }}>Y (meters)</label>
                  <input
                    type="number"
                    name="y"
                    value={formState.y}
                    onChange={handleInputChange}
                    min="-200"
                    max="200"
                    step="0.1"
                    style={{
                      width: "100%",
                      padding: "8px",
                      backgroundColor: "#202020",
                      border: "1px solid #444",
                      color: "#fff",
                      borderRadius: "4px"
                    }}
                  />
                </div>
              </div>
              <div style={{ display: "flex", gap: "12px" }}>
                <div style={{ flex: 1 }}>
                  <label style={{ fontSize: "0.8em", color: "#8c8c8c", display: "block", marginBottom: "4px" }}>Z (meters)</label>
                  <input
                    type="number"
                    name="z"
                    value={formState.z}
                    onChange={handleInputChange}
                    min="-1"
                    max="100"
                    step="0.1"
                    style={{
                      width: "100%",
                      padding: "8px",
                      backgroundColor: "#202020",
                      border: "1px solid #444",
                      color: "#fff",
                      borderRadius: "4px"
                    }}
                  />
                </div>
                <div style={{ flex: 1 }}>
                  <label style={{ fontSize: "0.8em", color: "#8c8c8c", display: "block", marginBottom: "4px" }}>Yaw (radians)</label>
                  <input
                    type="number"
                    name="yaw"
                    value={formState.yaw}
                    onChange={handleInputChange}
                    min="-3.1416"
                    max="3.1416"
                    step="0.01"
                    style={{
                      width: "100%",
                      padding: "8px",
                      backgroundColor: "#202020",
                      border: "1px solid #444",
                      color: "#fff",
                      borderRadius: "4px"
                    }}
                  />
                </div>
              </div>
              <div style={{ marginTop: "16px", display: "flex", justifyContent: "flex-end" }}>
                <button
                  type="button"
                  onClick={handleCapturePose}
                  disabled={capturing}
                  style={{
                    padding: "8px 16px",
                    backgroundColor: capturing ? "#444" : "#1976d2",
                    color: "#fff",
                    border: "none",
                    borderRadius: "4px",
                    cursor: capturing ? "not-allowed" : "pointer",
                    fontSize: "0.85em",
                    transition: "background-color 0.2s"
                  }}
                >
                  {capturing ? "Capturing..." : "Capture Pose from Simulator"}
                </button>
              </div>
            </div>
          )}
        </>
      )}
      <ModalRow type="buttons">
        <button
          type="button"
          id="create-combined-universe"
          onClick={() => handleCreate()}
        >
          Create World
        </button>
      </ModalRow>
    </>
  );
};

export default CreateCombinedPage;
