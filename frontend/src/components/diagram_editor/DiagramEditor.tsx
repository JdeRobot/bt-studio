import React, { useMemo, useState, useEffect, useRef } from "react";
import createEngine, { DiagramModel } from "@projectstorm/react-diagrams";

import {
  CanvasWidget,
  ZoomCanvasAction,
} from "@projectstorm/react-canvas-core";

import "./DiagramEditor.css";
import { BasicNodeModel } from "./nodes/basic_node/BasicNodeModel";
import { BasicNodeFactory } from "./nodes/basic_node/BasicNodeFactory"; // Import custom node factory

import { TagNodeModel } from "./nodes/tag_node/TagNodeModel";
import { TagNodeFactory } from "./nodes/tag_node/TagNodeFactory";

import NodeHeader from "./NodeHeader"; // Import HeaderMenu
import axios from "axios";

import { SimplePortFactory } from "./nodes/SimplePortFactory";
import { ChildrenPortModel } from "./nodes/basic_node/ports/children_port/ChildrenPortModel";
import { ParentPortModel } from "./nodes/basic_node/ports/parent_port/ParentPortModel";
import { OutputPortModel } from "./nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "./nodes/basic_node/ports/input_port/InputPortModel";
import { TagInputPortModel } from "./nodes/tag_node/ports/input_port/TagInputPortModel";
import { TagOutputPortModel } from "./nodes/tag_node/ports/output_port/TagOutputPortModel";
import EditActionModal from "./modals/EditActionModal.jsx";
import EditTagModal from "./modals/EditTagModal.jsx";

// Store action nodes and their inputs and outputs
interface ActionData {
  input: string[];
  output: string[];
  ids: string[];
  color: string;
}

const DiagramEditor = ({
  currentProjectname,
  setModelJson,
  setProjectChanges,
  gazeboEnabled,
  manager,
  actionNodesData,
  btOrder,
  openError,
}: {
  currentProjectname: any;
  setModelJson: any;
  setProjectChanges: any;
  gazeboEnabled: any;
  manager: any;
  actionNodesData: { [id: string]: ActionData };
  btOrder: string;
  openError: any;
}) => {
  const ref = useRef<any>(null);
  const [graphJson, setGraphJson] = useState(null);
  const [appRunning, setAppRunning] = useState(false);
  const [appLoaded, setAppLoaded] = useState(false);
  const [, setFocused] = useState(false);
  const [isEditActionModalOpen, setEditActionModalOpen] = useState(false);
  const [isEditTagModalOpen, setEditTagModalOpen] = useState(false);
  const [currentActionNode, setCurrentActionNode] = useState<any>(null);

  // Initial node position
  let lastMovedNodePosition = { x: 200, y: 200 };

  // Initialize state for last moved node ID
  const lastClickedNodeId = useRef<string>("");
  const forceNotReset = useRef(false);
  // Create the model
  const model = useRef(new DiagramModel());

  const handleLostFocus = (e: any) => {
    forceNotReset.current = true;
    if (
      e.relatedTarget &&
      (e.relatedTarget.id === "node-action-edit-button" ||
        e.relatedTarget.id === "node-action-delete-button" ||
        e.relatedTarget.id === "tagName" ||
        e.relatedTarget.className === "node-editor-button")
    ) {
      return;
    }
    setFocused(false);
    if (lastClickedNodeId.current !== "") {
      try {
        const node: any = model.current.getNode(lastClickedNodeId.current);
        if (node) {
          node.deselectNode();
        }
        setCurrentActionNode(null);
        lastClickedNodeId.current = "";
        setModelJson(JSON.stringify(model.current.serialize()));
      } catch {}
    }
  };

  const handleGainedFocus = (e: any) => {
    forceNotReset.current = true;
    setFocused(true);
  };

  // Listeners
  const attachPositionListener = (node: any) => {
    node.registerListener({
      positionChanged: (event: any) => {
        lastMovedNodePosition = event.entity.getPosition();
        setProjectChanges(true);
        setModelJson(JSON.stringify(model.current.serialize())); // Serialize and update model JSON
      },
    });
  };

  const attachLinkListener = (model: any) => {
    model.registerListener({
      linksUpdated: (event: any) => {
        const { link, isCreated } = event;
        link.registerListener({
          targetPortChanged: (link: any) => {
            if (isCreated) {
              const { sourcePort, targetPort } = link.entity;
              if (Object.keys(targetPort.getLinks()).length > 1) {
                model.removeLink(link.entity);
              } else if (
                sourcePort instanceof ChildrenPortModel &&
                !(targetPort instanceof ParentPortModel)
              ) {
                model.removeLink(link.entity);
              } else if (
                sourcePort instanceof TagOutputPortModel &&
                !(targetPort instanceof InputPortModel)
              ) {
                model.removeLink(link.entity);
              } else {
                model.clearSelection();
              }
            }
          },
        });
      },
    });
  };

  const attachClickListener = (node: any) => {
    node.registerListener({
      selectionChanged: (event: any) => {
        if (event.isSelected) {
          lastClickedNodeId.current = node.getID();
          node.selectNode();
        } else {
          node.deselectNode();
        }
      },
    });
  };

  const handleOpenEditActionModal = () => {
    if (lastClickedNodeId.current !== "") {
      const genericNode = model.current.getNode(lastClickedNodeId.current);
      const node = genericNode as BasicNodeModel;
      if (node && checkIfAction(node)) {
        forceNotReset.current = true;
        setCurrentActionNode(node);
        node.setSelected(false);
        setEditActionModalOpen(true);
      } else if (node && checkIfTag(node)) {
        forceNotReset.current = true;
        setCurrentActionNode(node);
        node.setSelected(false);
        setEditTagModalOpen(true);
      }
    }
  };

  const handleCloseEditActionModal = () => {
    setEditActionModalOpen(false);
    setEditTagModalOpen(false);
    setCurrentActionNode(null);
    lastClickedNodeId.current = "";
    setFocused(true);
    ref.current.focus();
  };

  const handleCloseEditTagModal = () => {
    setEditActionModalOpen(false);
    setEditTagModalOpen(false);
    setCurrentActionNode(null);
    lastClickedNodeId.current = "";
    setFocused(true);
    ref.current.focus();
    setModelJson(JSON.stringify(model.current.serialize()));
  };

  // Create the engine
  const engine = useMemo(() => {
    const newEngine = createEngine({
      registerDefaultZoomCanvasAction: false,
    });
    newEngine
      .getNodeFactories()
      .registerFactory(new BasicNodeFactory(handleOpenEditActionModal));
    newEngine
      .getNodeFactories()
      .registerFactory(new TagNodeFactory(handleOpenEditActionModal));
    newEngine
      .getPortFactories()
      .registerFactory(
        new SimplePortFactory("children", (config) => new ChildrenPortModel()),
      );
    newEngine
      .getPortFactories()
      .registerFactory(
        new SimplePortFactory("parent", (config) => new ParentPortModel()),
      );
    newEngine
      .getPortFactories()
      .registerFactory(
        new SimplePortFactory("output", (config) => new OutputPortModel("")),
      );
    newEngine
      .getPortFactories()
      .registerFactory(
        new SimplePortFactory("input", (config) => new InputPortModel("")),
      );
    newEngine
      .getPortFactories()
      .registerFactory(
        new SimplePortFactory(
          "tag output",
          (config) => new TagOutputPortModel(),
        ),
      );
    newEngine
      .getPortFactories()
      .registerFactory(
        new SimplePortFactory("tag input", (config) => new TagInputPortModel()),
      );
    const state: any = newEngine.getStateMachine().getCurrentState();
    state.dragNewLink.config.allowLooseLinks = false;
    newEngine
      .getActionEventBus()
      .registerAction(new ZoomCanvasAction({ inverseZoom: true }));
    return newEngine;
  }, []);

  // Update the graphJson when the project name changes
  useEffect(() => {
    // Deselect the current node
    setCurrentActionNode(null);
    if (lastClickedNodeId.current !== "") {
      let node: any = model.current.getNode(lastClickedNodeId.current);
      node.deselectNode();
      lastClickedNodeId.current = "";
    }
    if (currentProjectname) {
      // Only execute the API call if currentProjectname is set
      axios
        .get("/tree_api/get_project_graph/", {
          params: {
            project_name: currentProjectname,
          },
        })
        .then((response) => {
          if (response.data.success) {
            for (var member in actionNodesData) delete actionNodesData[member];
            // Set the model as the received json
            setGraphJson(response.data.graph_json);
            setModelJson(response.data.graph_json);
            // Create the new model
            model.current = new DiagramModel();
            // Reset the force reset state
            forceNotReset.current = false;
          } else {
            console.error(response.data.message);
          }
        })
        .catch((error) => {
          setGraphJson(null);
          forceNotReset.current = false;
          model.current = new DiagramModel();
        });
    }
    if (gazeboEnabled) {
      manager.terminate_application().then(() => {
        console.log("App terminated!");
        setAppRunning(false);
      });
    }
  }, [currentProjectname]);

  useEffect(() => {
    setCurrentActionNode(null);
    const nodes = model.current.getNodes(); // Assuming getNodes() method exists to retrieve all nodes
    nodes.forEach((node) => {
      if (checkIfAction(node)) {
        let castNode = node as BasicNodeModel;
        let name = castNode.getName();
        saveActionNodeData(node);
      }
    });
  }, [graphJson]);

  // Set the model in the engine ONLY on project change
  if (!forceNotReset.current) {
    if (graphJson === null) {
      const root_node = new BasicNodeModel("Tree Root", "rgb(0,204,0)");
      root_node.setPosition(200, 200);
      root_node.addChildrenPort("Children Port");
      model.current.addAll(root_node);
      setModelJson(JSON.stringify(model.current.serialize()));
      forceNotReset.current = true;
    } else {
      try {
        // Try parsing the JSON string
        model.current.deserializeModel(graphJson, engine);

        // After deserialization, attach listeners to each node
        const nodes = model.current.getNodes(); // Assuming getNodes() method exists to retrieve all nodes
        nodes.forEach((node) => {
          attachPositionListener(node);
          attachClickListener(node);
          node.setSelected(false);
        });
        attachLinkListener(model.current);
      } catch (e) {
        // Log the error for debugging
        console.error("An error occurred while parsing the JSON:", e);
      }
    }

    engine.setModel(model.current);
  }

  // Save action node data
  const saveActionNodeData = (node: any) => {
    let name = node.name;
    if (actionNodesData[name]) {
      actionNodesData[name]["ids"] = actionNodesData[name]["ids"].concat([
        node.getID(),
      ]);
      for (const inputs of actionNodesData[name]["input"]) {
        node.addInputPort(inputs);
      }
      for (const outputs of actionNodesData[name]["output"]) {
        node.addOutputPort(outputs);
      }
      node.setColor(actionNodesData[name]["color"]);
    } else {
      actionNodesData[name] = {
        input: [],
        output: [],
        ids: [node.getID()],
        color: node.getColor(),
      };
      for (let key in node.getPorts()) {
        if (key !== "parent") {
          if (node.getPorts()[key] instanceof InputPortModel) {
            node.addInputPort(key);
            actionNodesData[name]["input"] = actionNodesData[name][
              "input"
            ].concat([key]);
          } else if (node.getPorts()[key] instanceof OutputPortModel) {
            node.addOutputPort(key);
            actionNodesData[name]["output"] = actionNodesData[name][
              "output"
            ].concat([key]);
          }
        }
      }
    }
  };

  // Add the nodes default ports
  const addDefaultPorts = (node: any) => {
    console.log("Adding default ports");

    var nodeName = node.getName();
    if (nodeName === "RetryUntilSuccessful") node.addInputPort("num_attempts");
    else if (nodeName === "Repeat") node.addInputPort("num_cycles");
    else if (nodeName === "Delay") node.addInputPort("delay_ms");
  };

  const nodeTypeSelector = (nodeName: any) => {
    if (["Input port value", "Output port value"].includes(nodeName))
      addTagNode(nodeName);
    else addBasicNode(nodeName);
  };

  // Function to add a new node
  const addBasicNode = (nodeName: any) => {
    // Control parameters
    let nodeColor = "rgb(128,128,128)"; // Default color
    let hasInputPort = true;
    let hasOutputPort = true;
    let isAction = false;

    // Sequences
    if (
      ["Sequence", "ReactiveSequence", "SequenceWithMemory"].includes(nodeName)
    ) {
      nodeColor = "rgb(0,128,255)";
    }
    // Fallbacks
    else if (["Fallback", "ReactiveFallback"].includes(nodeName)) {
      nodeColor = "rgb(255,0,0)";
    }
    // Decorators
    else if (
      [
        "RetryUntilSuccessful",
        "Inverter",
        "ForceSuccess",
        "ForceFailure",
        "KeepRunningUntilFailure",
        "Repeat",
        "RunOnce",
        "Delay",
      ].includes(nodeName)
    ) {
      nodeColor = "rgb(255,153,51)";
    }
    // Actions
    else {
      nodeColor = "rgb(128,0,128)";
      hasOutputPort = false;
      isAction = true;
    }

    // Create node
    const newNode = new BasicNodeModel(nodeName, nodeColor);

    if (isAction) {
      saveActionNodeData(newNode);
      setCurrentActionNode(newNode);
    }

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    lastClickedNodeId.current = newNode.getID();

    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;

    // Add ports
    if (hasInputPort) newNode.addParentPort("Parent Port");
    if (hasOutputPort) newNode.addChildrenPort("Children Port");
    addDefaultPorts(newNode);

    // Add the node to the model
    if (model.current) {
      model.current.addNode(newNode);
      newNode.selectNode();
      setProjectChanges(true);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.current.serialize()));
    }
    forceNotReset.current = true;
  };

  const addTagNode = (nodeName: any) => {
    const newNode = new TagNodeModel("value", "rgb(128,128,128)");

    if (nodeName === "Input port value") newNode.addOutputPort();
    else newNode.addInputPort();

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    lastClickedNodeId.current = newNode.getID();

    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;

    // Add the node to the model
    if (model.current) {
      model.current.addNode(newNode);
      newNode.selectNode();
      setProjectChanges(true);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.current.serialize()));
    }
    forceNotReset.current = true;
  };

  const deleteLastClickedNode = () => {
    forceNotReset.current = true;
    if (model.current && lastClickedNodeId.current) {
      const node: any = model.current.getNode(lastClickedNodeId.current);
      if (node) {
        if (checkIfAction(node)) {
          actionNodesData[node!.getName()]["ids"] = actionNodesData[
            node!.getName()
          ]["ids"].filter((obj) => obj !== lastClickedNodeId.current);
        }
        node.remove();
        setProjectChanges(true);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      }
      lastClickedNodeId.current = "";
    }
  };

  const checkIfAction = (node: any) => {
    var name = node.name;

    if (node.getOptions().type === "tag") {
      return false;
    }

    // Check if the node is a user written action
    return ![
      "Sequence",
      "ReactiveSequence",
      "SequenceWithMemory",
      "Fallback",
      "ReactiveFallback",
      "RetryUntilSuccessful",
      "Inverter",
      "ForceSuccess",
      "ForceFailure",
      "KeepRunningUntilFailure",
      "Repeat",
      "RunOnce",
      "Delay",
      "Input port value",
      "Output port value",
      "Tree Root",
    ].includes(name);
  };

  const checkIfTag = (node: any) => {
    return node.getOptions().type === "tag";
  };

  const setColorActionNode = (r: number, g: number, b: number) => {
    currentActionNode.setColor(
      "rgb(" + Math.round(r) + "," + Math.round(g) + "," + Math.round(b) + ")",
    );
    actionNodesData[currentActionNode.getName()]["color"] =
      currentActionNode.getColor();
    console.log(actionNodesData);
    let name = currentActionNode.getName();
    for (const nodesId of actionNodesData[name]["ids"]) {
      let genericActionNode = model.current.getNode(nodesId);
      let actionNode = genericActionNode as BasicNodeModel;
      if (actionNode) {
        actionNode.setColor(actionNodesData[name]["color"]);
      } else {
        actionNodesData[name]["ids"] = actionNodesData[name]["ids"].filter(
          (obj) => obj !== nodesId,
        );
      }
    }
    engine.repaintCanvas();
    setModelJson(JSON.stringify(model.current.serialize()));
  };

  const addInputPort = (portName: string) => {
    if (currentActionNode) {
      let name = currentActionNode.getName();
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        if (portName !== null) {
          // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.addInputPort(portName);
          actionNodesData[name]["input"] = actionNodesData[name][
            "input"
          ].concat([portName]);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[name]["ids"]) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              if (actionNode) {
                actionNode.addInputPort(portName);
              } else {
                actionNodesData[name]["ids"] = actionNodesData[name][
                  "ids"
                ].filter((obj) => obj !== nodesId);
              }
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        openError("Ports can only be added to action nodes");
      }
    }
  };

  const addOutputPort = (portName: string) => {
    if (currentActionNode) {
      let name = currentActionNode.getName();
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        if (portName !== null) {
          // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.addOutputPort(portName);
          actionNodesData[name]["output"] = actionNodesData[name][
            "output"
          ].concat([portName]);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[name]["ids"]) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              if (actionNode) {
                actionNode.addOutputPort(portName);
              } else {
                actionNodesData[name]["ids"] = actionNodesData[name][
                  "ids"
                ].filter((obj) => obj !== nodesId);
              }
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        openError("Ports can only be added to action nodes");
      }
    }
  };

  const deleteInputPort = (port: InputPortModel, portName: string) => {
    if (currentActionNode) {
      let name = currentActionNode.getName();
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        if (port !== null) {
          // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.removeInputPort(port);
          actionNodesData[name]["input"] = actionNodesData[name][
            "input"
          ].filter((item) => item !== portName);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[name]["ids"]) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              if (actionNode) {
                actionNode.removeInputPort(port);
              } else {
                actionNodesData[name]["ids"] = actionNodesData[name][
                  "ids"
                ].filter((obj) => obj !== nodesId);
              }
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        openError("Ports can only be removed from action nodes");
      }
    }
  };

  const deleteOutputPort = (port: OutputPortModel, portName: string) => {
    if (currentActionNode) {
      let name = currentActionNode.getName();
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        if (port !== null) {
          // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.removeOutputPort(port);
          actionNodesData[name]["output"] = actionNodesData[name][
            "output"
          ].filter((item) => item !== portName);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[name]["ids"]) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              if (actionNode) {
                actionNode.removeOutputPort(port);
              } else {
                actionNodesData[name]["ids"] = actionNodesData[name][
                  "ids"
                ].filter((obj) => obj !== nodesId);
              }
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        openError("Ports can only be removed from action nodes");
      }
    }
  };

  const zoomToFit = () => {
    engine.zoomToFitNodes({ margin: 50 });
  };

  const generateApp = () => {
    if (model.current) {
      const str = JSON.stringify(model.current.serialize());

      fetch("/tree_api/generate_app/", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          app_name: currentProjectname,
          content: str,
          bt_order: btOrder,
        }),
      })
        .then((response) => {
          if (!response.ok) {
            return response.json().then((data) => {
              throw new Error(data.message || "An error occurred.");
            });
          }
          return response.blob();
        })
        .then((blob) => {
          // Get the application
          const url = window.URL.createObjectURL(blob);
          const a = document.createElement("a");
          a.style.display = "none";
          a.href = url;
          a.download = `${currentProjectname}.zip`;
          document.body.appendChild(a);
          a.click();
          window.URL.revokeObjectURL(url);
        })
        .catch((error) => {
          console.error("Error:", error);
        });
    }
  };

  const runApp = () => {
    if (gazeboEnabled) {
      if (!appRunning) {
        if (!appLoaded) {
          const tree_graph = JSON.stringify(model.current.serialize());
          fetch("/tree_api/get_simplified_app/", {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
            },
            body: JSON.stringify({
              app_name: currentProjectname,
              content: tree_graph,
              bt_order: btOrder,
            }),
          })
            .then((response) => {
              if (!response.ok) {
                return response.json().then((data) => {
                  throw new Error(data.message || "An error occurred.");
                });
              }
              return response.blob();
            })
            .then((blob) => {
              var reader = new FileReader();
              reader.readAsDataURL(blob);
              reader.onloadend = function () {
                var base64data = reader.result;
                // console.log(base64data);

                // Send the zip
                manager
                  .run({ type: "bt-studio", code: base64data })
                  .then(() => {
                    console.log("App running!");
                    setAppRunning(true);
                    setAppLoaded(true);
                  })
                  .catch((response: any) => {
                    let linterMessage = JSON.stringify(
                      response.data.message,
                    ).split("\\n");
                    alert(`Received linter message: ${linterMessage}`);
                  });
              };
            })
            .catch((error) => {
              console.error("Error:", error);
            });
        } else {
          // Send the zip
          manager
            .resume()
            .then(() => {
              console.log("App resumed!");
              setAppRunning(true);
            })
            .catch((response: any) => {
              let linterMessage = JSON.stringify(response.data.message).split(
                "\\n",
              );
              alert(`Received linter message: ${linterMessage}`);
            });
        }
      } else {
        manager.pause().then(() => {
          console.log("App paused!");
          setAppRunning(false);
        });
      }
    } else {
      console.log("Gazebo is not up!");
    }
  };

  const resetApp = () => {
    if (gazeboEnabled) {
      manager.terminate_application().then(() => {
        console.log("App reseted!");
        setAppRunning(false);
        setAppLoaded(false);
      });
    } else {
      console.log("Gazebo is not up!");
    }
  };

  return (
    <div id="diagram-editor">
      <NodeHeader
        onNodeTypeSelected={nodeTypeSelector}
        onDeleteNode={deleteLastClickedNode}
        onEditAction={handleOpenEditActionModal}
        onGenerateApp={generateApp}
        onRunApp={runApp}
        onResetApp={resetApp}
        isAppRunning={appRunning}
        currentProjectname={currentProjectname}
        zoomToFit={zoomToFit}
      />
      <div
        tabIndex={0}
        ref={ref}
        onBlur={(e) => handleLostFocus(e)}
        onFocus={(e) => handleGainedFocus(e)}
        id="diagram-view"
      >
        <CanvasWidget className="canvas" engine={engine} />
      </div>
      <button
        className="node-action-button"
        style={{
          position: "relative",
          bottom: "25px",
          marginLeft: "auto",
          backgroundColor: "transparent",
        }}
        title={"BT Order: " + btOrder}
      >
        <svg
          className="w-6 h-6 text-gray-800 dark:text-white"
          aria-hidden="true"
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          fill="none"
          viewBox="0 0 24 24"
        >
          {btOrder === "bottom-to-top" ? (
            <path
              stroke="var(--background)"
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth="2"
              d="M12 6v13m0-13 4 4m-4-4-4 4"
            />
          ) : (
            <path
              stroke="var(--background)"
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth="2"
              d="M12 19V5m0 14-4-4m4 4 4-4"
            />
          )}
        </svg>
      </button>
      <EditActionModal
        isOpen={isEditActionModalOpen}
        onClose={handleCloseEditActionModal}
        currentActionNode={currentActionNode}
        setColorActionNode={setColorActionNode}
        addInputPort={addInputPort}
        addOutputPort={addOutputPort}
        deleteInputPort={deleteInputPort}
        deleteOutputPort={deleteOutputPort}
      />
      <EditTagModal
        isOpen={isEditTagModalOpen}
        onClose={handleCloseEditTagModal}
        currentActionNode={currentActionNode}
      />
    </div>
  );
};

export default React.memo(DiagramEditor);
