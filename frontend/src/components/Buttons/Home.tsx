import { useState } from "react";
import { StyledHeaderButton } from "../../styles/headers/HeaderMenu.styles";
import { useBtTheme } from "../../contexts/BtThemeContext";
import HomeRoundedIcon from "@mui/icons-material/HomeRounded";
import ProjectModal from "../HeaderMenu/modals/ProjectModal";
import { CommsManager, states } from "jderobot-commsmanager";
import { useError } from "jderobot-ide-interface";
import { createProject } from "../../api_helper/TreeWrapper";

const HomeButton = ({
  project,
  manager,
  setProject,
  setAppRunning,
}: {
  project: string;
  manager: CommsManager | null;
  setProject: Function;
  setAppRunning: Function;
}) => {
  const theme = useBtTheme();
  const { warning, error } = useError();
  const [existingProjects, setExistingProjects] = useState<string[]>([]);
  const [isProjectModalOpen, setProjectModalOpen] = useState(true);

  const terminateUniverse = async () => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      );
      return;
    }

    if (manager.getUniverse() === "") {
      return;
    }

    const state = manager.getState();

    if (state === states.IDLE || state === states.CONNECTED) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected.",
      );
      return;
    }

    if (state === states.RUNNING || state === states.PAUSED) {
      await manager.terminateApplication();
      setAppRunning(false);
    }

    if (manager.getState() === states.TOOLS_READY) {
      await manager.terminateTools();
    }

    if (manager.getState() === states.WORLD_READY) {
      await manager.terminateUniverse();
    }
  };

  const onCreateProject = async (projectName: string) => {
    try {
      await createProject(projectName);
      setProject(projectName);
      console.log("Project created successfully");
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Error creating project: " + e.message);
        error("Error creating project: " + e.message);
      }
    }
  };

  const onChangeProject = async (projectName: string) => {
    if (existingProjects.includes(projectName)) {
      // Project exists, proceed to change
      setProject(projectName);

      // Terminate the universe
      if (manager?.getUniverse()) {
        await terminateUniverse();
      }
      console.log(`Switched to project ${projectName}`);
      console.log("Universe terminated!");
    } else {
      console.error(`The project ${projectName} does not exist`);
      error(`The project ${projectName} does not exist`);
    }
  };

  const onOpenProjectModal = (e: any) => {
    setProjectModalOpen(true);
  };

  const onCloseProjectModal = async (projectName: string) => {
    if (projectName) {
      await onChangeProject(projectName);
    }
    setProjectModalOpen(false);
  };

  return (
    <>
      <ProjectModal
        isOpen={isProjectModalOpen}
        onSubmit={(data: unknown) => {}}
        onClose={onCloseProjectModal}
        currentProject={project}
        existingProjects={existingProjects}
        setExistingProjects={setExistingProjects}
        createProject={onCreateProject}
      />
      <StyledHeaderButton
        bgColor={theme.palette.primary}
        hoverColor={theme.palette.secondary}
        roundness={theme.roundness}
        id="return-academy"
        title="Return to Home"
        onClick={onOpenProjectModal}
      >
        <HomeRoundedIcon htmlColor={theme.palette.text} />
      </StyledHeaderButton>
    </>
  );
};

export default HomeButton;
