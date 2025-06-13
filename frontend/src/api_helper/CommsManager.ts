import { v4 as uuidv4 } from "uuid";
import { publish } from "../components/helper/TreeEditorHelper";

// Define types for the resolve and reject functions
type PromiseHandlers = {
  resolve: (value?: any) => void;
  reject: (reason?: any) => void;
};

interface ManagerMsg {
  id: string;
  command: string;
  data: any;
}

interface WorldConfig {
  name: string;
  launch_file_path: string;
  ros_version: string;
  world: string;
  zip?: unknown;
}

interface RobotConfig {
  name: string | null;
  launch_file_path: string | null;
  ros_version: string | null;
  visualization: string | null;
  world: string | null;
  start_pose: string | null;
}

interface UniverseConfig {
  name: string;
  world: WorldConfig;
  robot: RobotConfig;
}

type Events = string | string[];
export default class CommsManager {
  private static instance: CommsManager | undefined;
  private ws: WebSocket;
  private observers: { [id: string]: Function[] } = {};
  private pendingPromises: Map<string, PromiseHandlers> = new Map();
  private static state: string = "idle";
  private static universe?: string;
  private static hostData?: {
    gpu_avaliable: string;
    robotics_backend_version: string;
    ros_version: string;
  };

  // Private constructor to only allow single instatiation
  private constructor(address: string) {
    this.ws = new WebSocket(address);
    this.subscribe(events.STATE_CHANGED, this.setManagerState);
    this.subscribeOnce(events.INTROSPECTION, this.setHostData);

    // Message callback
    this.ws.onmessage = (event) => {
      const msg = JSON.parse(event.data);

      // Check if the message ID exists in the pending promises map
      const handlers = this.pendingPromises.get(msg.id);
      if (handlers) {
        const { resolve, reject } = handlers;

        if (msg.command === "ack") {
          resolve(msg);
        } else if (msg.command === "error") {
          reject(msg);
        }

        // Clean up after handling the response
        this.pendingPromises.delete(msg.id);
      } else {
        // Look if there are any subscribers
        const subscriptions = this.observers[msg.command] || [];
        let length = subscriptions.length;
        while (length--) {
          subscriptions[length](msg);
        }
      }
    };

    // Closing callback
    this.ws.onclose = (e) => {
      if (e.wasClean) {
        console.log(
          `Connection with ${address} closed, all suscribers cleared`
        );
      } else {
        console.log(`Connection with ${address} interrupted`);
      }
    };
  }

  // Singleton behavior
  public static getInstance(): CommsManager {
    if (!CommsManager.instance) {
      CommsManager.instance = new CommsManager("ws://127.0.0.1:7163");
    }
    return CommsManager.instance;
  }

  public static deleteInstance() {
    if (CommsManager.instance) {
      delete CommsManager.instance;
      CommsManager.instance = undefined;
    }
  }

  public subscribe = (events: Events, callback: Function) => {
    if (typeof events === "string") {
      events = [events];
    }
    for (let i = 0, length = events.length; i < length; i++) {
      this.observers[events[i]] = this.observers[events[i]] || [];
      this.observers[events[i]].push(callback);
    }
  };

  public subscribeOnce = (event: Events, callback: Function) => {
    this.subscribe(event, (response: any) => {
      callback(response);
      this.unsubscribe(event, callback);
    });
  };

  public unsubscribe = (events: Events, callback: Function) => {
    if (typeof events === "string") {
      events = [events];
    }
    for (let i = 0, length = events.length; i < length; i++) {
      this.observers[events[i]] = this.observers[events[i]] || [];
      this.observers[events[i]].splice(
        this.observers[events[i]].indexOf(callback)
      );
    }
  };

  public unsuscribeAll = () => {
    for (const event in this.observers) {
      this.observers[event].length = 0;
    }
  };

  // Send messages and manage promises
  public async send(message: string, data?: Object): Promise<any> {
    const id = uuidv4();

    // Reject with an Error directly if unable to connect
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error("WebSocket not connected"));
    }

    // Return a new Promise that handles the message response
    return new Promise((resolve, reject) => {
      // Store the promise's resolve and reject in the map
      this.pendingPromises.set(id, { resolve, reject });

      // Send the message
      const msg = JSON.stringify({
        id: id,
        command: message,
        data: data,
      });
      this.ws.send(msg);
    });
  }

  private setManagerState(msg: ManagerMsg) {
    publish("CommsManagerStateChange", {state: msg.data.state})
    CommsManager.state = msg.data.state;
    if (msg.data.state === "connected") {
      CommsManager.universe = undefined
    }
  }

  public getState() {
    return CommsManager.state;
  }

  private setHostData(msg: ManagerMsg) {
    CommsManager.hostData = msg.data;
  }

  public getHostData() {
    return CommsManager.hostData;
  }

  public getUniverse() {
    return CommsManager.universe;
  }

  // Connect to the ws
  public connect() {
    return this.send("connect");
  }

  public launchWorld(cfg: UniverseConfig) {
    CommsManager.universe = cfg.name
    return this.send("launch_world", cfg);
  }

  public prepareVisualization(
    visualization_type: string,
    visualization_config: string | null
  ) {
    if (visualization_config === null || visualization_config === undefined) {
      visualization_config = "None";
    }
    return this.send("prepare_visualization", {
      type: visualization_type,
      file: visualization_config,
    });
  }

  public run(cfg: Object) {
    return this.send("run_application", cfg);
  }

  public stop() {
    return this.send("stop");
  }

  public pause() {
    return this.send("pause");
  }

  public resume() {
    return this.send("resume");
  }

  public reset() {
    return this.send("reset");
  }

  public terminateApplication() {
    return this.send("terminate_application");
  }

  public terminateVisualization() {
    return this.send("terminate_visualization");
  }

  public terminateUniverse() {
    return this.send("terminate_universe");
  }

  public disconnect() {
    return this.send("disconnect");
  }

  public style_check(code: string) {
    return this.send("style_check", { code: code });
  }

  public code_format(code: string) {
    return this.send("code_format", { code: code });
  }

  public code_analysis(code: string, disable_errors: string[]) {
    return this.send("code_analysis", {
      code: code,
      disable_errors: disable_errors,
    });
  }

  public code_autocomplete(code: string, line: number, col: number) {
    return this.send("code_autocomplete", { code: code, line: line, col: col });
  }
}

const events = {
  RESPONSES: ["ack", "error"],
  UPDATE: "update",
  STATE_CHANGED: "state-changed",
  INTROSPECTION: "introspection",
  CODE_FORMAT: "code-format",
  CODE_ANALYSIS: "code-analysis",
  CODE_AUTOCOMPLETE: "code-autocomplete",
};

const states = {
  INTROSPECTION: "idle",
  CODE_FORMAT: "connected",
  CODE_ANALYSIS: "world_ready",
  RESPONSES: "visualization_ready",
  UPDATE: "application_running",
  STATE_CHANGED: "paused",
};
