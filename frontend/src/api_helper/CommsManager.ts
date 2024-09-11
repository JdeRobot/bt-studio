import { v4 as uuidv4 } from "uuid";

// Define types for the resolve and reject functions
type PromiseHandlers = {
  resolve: (value?: any) => void;
  reject: (reason?: any) => void;
};

export default class CommsManager {
  private static instance: CommsManager;
  private ws: WebSocket;
  private pendingPromises: Map<string, PromiseHandlers> = new Map();

  // Private constructor to only allow single instatiation
  private constructor(address: string) {
    this.ws = new WebSocket(address);

    // Message callback
    this.ws.onmessage = (event) => {
      const msg = JSON.parse(event.data);
      console.log(msg);

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
      }
    };

    // Closing callback
    this.ws.onclose = (e) => {
      if (e.wasClean) {
        console.log(
          `Connection with ${address} closed, all suscribers cleared`,
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

  // Connect to the ws
  public connect() {
    return this.send("connect");
  }

  public launchWorld(cfg: Object) {
    return this.send("launch_world", cfg);
  }

  public prepareVisualization(visualization_type: string) {
    return this.send("prepare_visualization", visualization_type);
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
}
