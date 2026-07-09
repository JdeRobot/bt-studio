export type BTStatus = "RUNNING" | "SUCCESS" | "FAILURE" | "INVALID";

type Scene = {
  name: string;
  launch_file_path: string;
  ros_version: string;
  type: string;
  tools_config: string;
  zip?: unknown;
};

type Robot = {
  name: string;
  launch_file_path: string;
  ros_version: string;
  type: string;
  start_pose: string;
  entity: string;
  extra_config: string;
};

type WorldConfig = {
  scene: Scene;
  robot: Robot[];
  tools: string[];
  tools_config: { [key: string]: string };
};

export type BTWorldData = {
  name: string;
  isCustom: boolean;
  config: WorldConfig;
};
