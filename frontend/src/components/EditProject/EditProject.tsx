import React, { useEffect, useState } from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledSectionName } from "BtStyles/ProjectMenu/ProjectMenu.styles";
import ToggleButton from "@mui/material/ToggleButton";

import {
  StyledButton,
  StyledTextField,
  StyledToggleButtonGroup,
} from "BtStyles/CreateProject/CreateProject.styles";
import {
  createProject,
  getProjectConfigRaw,
  getProjectInfo,
} from "BtApi/TreeWrapper";
import { useError } from "jderobot-ide-interface";
import { useNavigate } from "react-router-dom";
import {
  GazeboIcon,
  RvizIcon,
  TerminalIcon,
  TreeMonitorIcon,
  WebGUIIcon,
} from "BtIcons";

const Menu = ({ projId }: { projId: string }) => {
  const theme = useBtTheme();
  const navigate = useNavigate();
  const { error } = useError();
  const [name, setName] = useState<string | undefined>(undefined);
  const [loading, setLoading] = useState(false);
  const [tools, setTools] = useState(() => ["Simulator", "Terminal"]);

  const handleTools = (
    event: React.MouseEvent<HTMLElement>,
    newTools: string[],
  ) => {
    setTools(newTools);
  };

  const handleName = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = event.target.value;
    setName(value);
  };

  function sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  const onCreateProject = async () => {
    if (name === undefined) {
      error("Project Name is missing");
      return;
    }
    setLoading(true);
    try {
      console.log("Edit project", name);
      // await createProject(name);
      await sleep(10000);
      navigate("..");
      console.log("Project edited successfully");
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Error editing project: " + e.message);
        error("Error editing project: " + e.message);
      }
    }
    setLoading(false);
  };

  const getInfo = async (id: string) => {
    const info = await getProjectInfo(id);
    setName(info.name);
    // const settings = await getProjectConfigRaw(id);
    // setTools(info.tools);
  };

  useEffect(() => {
    getInfo(projId);
  }, []);

  return (
    <>
      <StyledSectionName
        color={theme.palette.text}
        style={{ textAlign: "center" }}
      >
        Project Name
      </StyledSectionName>
      <div style={{ display: "flex", justifyContent: "center" }}>
        <StyledTextField
          id="project-name-input"
          onChange={handleName}
          bg={theme.palette.bg}
          errBg={theme.palette.error!}
          selBg={theme.palette.primary!}
          text={theme.palette.text!}
          roundness={theme.roundness ?? 1}
          label="Project Name"
          variant="filled"
          required
          error={name === "a"}
          value={name}
          focused={projId !== undefined}
        />
      </div>
      <StyledSectionName
        color={theme.palette.text}
        style={{ textAlign: "center" }}
      >
        Change Tools
      </StyledSectionName>
      <StyledToggleButtonGroup
        value={tools}
        onChange={handleTools}
        bg={theme.palette.bg}
        selBg={theme.palette.primary!}
        text={theme.palette.text!}
        roundness={theme.roundness ?? 1}
      >
        <ToggleButton value="WebGUI" disabled>
          <WebGUIIcon htmlColor={theme.palette.text} />
          <label>WebGUI</label>
        </ToggleButton>
        <ToggleButton value="RVIZ2">
          <RvizIcon htmlColor={theme.palette.text} />
          <label>RVIZ2</label>
        </ToggleButton>
        <ToggleButton value="Simulator">
          <GazeboIcon htmlColor={theme.palette.text} />
          <label>Simulator</label>
        </ToggleButton>
        <ToggleButton value="Terminal">
          <TerminalIcon htmlColor={theme.palette.text} />
          <label>Terminal</label>
        </ToggleButton>
        <ToggleButton value="BT Monitor">
          <TreeMonitorIcon htmlColor={theme.palette.text} />
          <label>BT Monitor</label>
        </ToggleButton>
      </StyledToggleButtonGroup>
      <div
        style={{
          display: "flex",
          alignItems: "center",
          flexDirection: "column",
        }}
      >
        <StyledButton
          variant="contained"
          bg={theme.palette.bg}
          text={theme.palette.text!}
          roundness={theme.roundness ?? 1}
          onClick={onCreateProject}
          loading={loading}
          loadingPosition="end"
          id="submit-edited-project"
        >
          Edit
        </StyledButton>
      </div>
    </>
  );
};

export default Menu;
