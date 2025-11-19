import React, { useEffect, useState } from "react";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledSectionName } from "BtStyles/ProjectMenu/ProjectMenu.styles";
import ToggleButton from "@mui/material/ToggleButton";

import ClearAllRoundedIcon from "@mui/icons-material/ClearAllRounded";
import AddCircleOutlineRoundedIcon from "@mui/icons-material/AddCircleOutlineRounded";
import TerminalRoundedIcon from "@mui/icons-material/TerminalRounded";
import VideoCameraBackRoundedIcon from "@mui/icons-material/VideoCameraBackRounded";
import AccountTreeRoundedIcon from "@mui/icons-material/AccountTreeRounded";
import ImportantDevicesRoundedIcon from "@mui/icons-material/ImportantDevicesRounded";
import PrecisionManufacturingRoundedIcon from "@mui/icons-material/PrecisionManufacturingRounded";
import {
  StyledButton,
  StyledTextField,
  StyledToggleButtonGroup,
} from "BtStyles/CreateProject/CreateProject.styles";
import { createProject, getProjectInfo } from "BtApi/TreeWrapper";
import { useError } from "jderobot-ide-interface";
import { useNavigate } from "react-router-dom";

const Menu = ({ projId }: { projId?: string }) => {
  const theme = useBtTheme();
  const navigate = useNavigate();
  const { error } = useError();
  const [name, setName] = useState<string | undefined>(undefined);
  const [template, setTemplate] = useState("Behaviour Tree");
  const [loading, setLoading] = useState(false);
  const [tools, setTools] = useState(() => ["Simulator", "Terminal"]);

  const handleTools = (
    event: React.MouseEvent<HTMLElement>,
    newTools: string[]
  ) => {
    setTools(newTools);
  };

  const handleTemplate = (
    event: React.MouseEvent<HTMLElement>,
    newTemplate: string
  ) => {
    setTemplate(newTemplate);
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
    setLoading(true)
    try {
      await createProject(name);
      await sleep(10000);
      navigate("/home");
      console.log("Project created successfully");
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Error creating project: " + e.message);
        error("Error creating project: " + e.message);
      }
    }
    setLoading(false)
  };

  const getInfo = async (id: string) => {
    const info = await getProjectInfo(id);
    setName(info.name + " (copy)");
  };

  useEffect(() => {
    if (projId) {
      getInfo(projId);
    }
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
        Select starting template
      </StyledSectionName>
      <StyledToggleButtonGroup
        value={template}
        onChange={handleTemplate}
        exclusive
        bg={theme.palette.bg}
        selBg={theme.palette.primary!}
        text={theme.palette.text!}
        roundness={theme.roundness ?? 1}
      >
        <ToggleButton value="Empty" disabled>
          <AddCircleOutlineRoundedIcon htmlColor={theme.palette.text} />
          <label>Empty</label>
        </ToggleButton>
        <ToggleButton value="Sequential + Iterative" disabled>
          <ClearAllRoundedIcon htmlColor={theme.palette.text} />
          <label>Sequential + Iterative</label>
        </ToggleButton>
        <ToggleButton value="Behaviour Tree">
          <AccountTreeRoundedIcon htmlColor={theme.palette.text} />
          <label>Behaviour Tree</label>
        </ToggleButton>
      </StyledToggleButtonGroup>
      <StyledSectionName
        color={theme.palette.text}
        style={{ textAlign: "center" }}
      >
        Select Tools
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
          <ImportantDevicesRoundedIcon htmlColor={theme.palette.text} />
          <label>WebGUI</label>
        </ToggleButton>
        <ToggleButton value="RVIZ2">
          <PrecisionManufacturingRoundedIcon htmlColor={theme.palette.text} />
          <label>RVIZ2</label>
        </ToggleButton>
        <ToggleButton value="Simulator">
          <VideoCameraBackRoundedIcon htmlColor={theme.palette.text} />
          <label>Simulator</label>
        </ToggleButton>
        <ToggleButton value="Terminal">
          <TerminalRoundedIcon htmlColor={theme.palette.text} />
          <label>Terminal</label>
        </ToggleButton>
        <ToggleButton value="BT Monitor">
          <AccountTreeRoundedIcon htmlColor={theme.palette.text} />
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
        >
          Create
        </StyledButton>
      </div>
    </>
  );
};

export default Menu;
