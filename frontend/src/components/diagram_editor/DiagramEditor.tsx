import React, { useMemo, useState, useEffect, useRef } from 'react';
import createEngine, { 
  DefaultLinkModel, 
  DefaultNodeModel,
  DiagramEngine,
  DiagramModel 
} from '@projectstorm/react-diagrams';

import {
  CanvasWidget
} from '@projectstorm/react-canvas-core';

import './DiagramEditor.css';
import { BasicNodeModel } from './nodes/basic_node/BasicNodeModel';
import { BasicNodeFactory } from './nodes/basic_node/BasicNodeFactory'; // Import custom node factory

import { TagNodeModel } from './nodes/tag_node/TagNodeModel';
import { TagNodeFactory } from './nodes/tag_node/TagNodeFactory';

import NodeHeader from './NodeHeader'; // Import HeaderMenu
import axios from 'axios';

import { SimplePortFactory } from './nodes/SimplePortFactory';
import { ChildrenPortModel } from './nodes/basic_node/ports/children_port/ChildrenPortModel';
import { ParentPortModel } from './nodes/basic_node/ports/parent_port/ParentPortModel';
import { OutputPortModel } from './nodes/basic_node/ports/output_port/OutputPortModel';
import { InputPortModel } from './nodes/basic_node/ports/input_port/InputPortModel';
import { TagInputPortModel } from './nodes/tag_node/ports/input_port/TagInputPortModel';
import { TagOutputPortModel } from './nodes/tag_node/ports/output_port/TagOutputPortModel';
import EditActionModal from './EditActionModal.jsx';

const DiagramEditor = ({currentProjectname, setModelJson, setProjectChanges, gazeboEnabled, manager} : {currentProjectname : any, setModelJson : any, setProjectChanges:any, gazeboEnabled:any, manager:any}) => {

  const [graphJson, setGraphJson] = useState(null);
  const [appRunning, setAppRunning] = useState(false);
  const [isEditActionModalOpen, setEditActionModalOpen] = useState(false);
  const [currentActionNode, setCurrentActionNode] = useState<any>(null);
  const [modelProjectName, setModelProjectName] = useState("");
  const [actionNodesData, setActionNodesData] = useState<{ [id: string]: ActionData; }>({});

  // Initial node position
  let lastMovedNodePosition = { x: 200, y: 200 };
  
  // Initialize state for last moved node ID
  // let lastClickedNodeId = "";
  const lastClickedNodeId = useRef<string>("");
  const forceNotReset = useRef(false);
  // Create the model
  const model = useRef(new DiagramModel());

  // Store action nodes and their inputs and outputs
  interface ActionData {
    input: string[];
    output: string[];
    ids: string[];
    color: string;
  }
  // let actionNodesData: { [id: string]: ActionData; } = {};

  // Listeners
  const attachPositionListener = (node:any) => {
    node.registerListener({
      positionChanged: (event:any) => {
        lastMovedNodePosition = event.entity.getPosition();
        setProjectChanges(true);
        setModelJson(JSON.stringify(model.current.serialize())); // Serialize and update model JSON
      },
    });
  };

  const attachLinkListener = (node:any) => {
    node.registerListener({
      linksUpdated: (event:any) => {
        if (event.isCreated) {
          setProjectChanges(true);
          setModelJson(JSON.stringify(model.current.serialize())); // Update when a new link is created
        }
      },
    });
  };

  const attachClickListener = (node:any) => {
    node.registerListener({
      selectionChanged: (event:any) => {
        if (event.isSelected) {
          lastClickedNodeId.current = node.getID();
          node.selectNode();
        } else {
          node.deselectNode();
        }
      },
    });
  };

  // Create the engine
  const engine = useMemo(() => {
    const newEngine = createEngine();
    newEngine.getNodeFactories().registerFactory(new BasicNodeFactory());
    newEngine.getNodeFactories().registerFactory(new TagNodeFactory());
    newEngine.getPortFactories().registerFactory(new SimplePortFactory('children', (config) => new ChildrenPortModel()));
    newEngine.getPortFactories().registerFactory(new SimplePortFactory('parent', (config) => new ParentPortModel()));
    newEngine.getPortFactories().registerFactory(new SimplePortFactory('output', (config) => new OutputPortModel("")));
    newEngine.getPortFactories().registerFactory(new SimplePortFactory('input', (config) => new InputPortModel("")));
    newEngine.getPortFactories().registerFactory(new SimplePortFactory('tag output', (config) => new TagOutputPortModel()));
    newEngine.getPortFactories().registerFactory(new SimplePortFactory('tag input', (config) => new TagInputPortModel()));
    return newEngine;
  }, []);

  // Update the graphJson when the project name changes
  useEffect(() => {    
    if (currentProjectname) { // Only execute the API call if currentProjectname is set
      axios.get('/tree_api/get_project_graph/', {
        params: {
          project_name: currentProjectname
        }
      })
      .then(response => {
        if (response.data.success) {
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
      .catch(error => {
        setGraphJson(null);
      });
    }
  
  }, [currentProjectname]);

  useEffect(() => {
    const nodes = model.current.getNodes();  // Assuming getNodes() method exists to retrieve all nodes
    nodes.forEach((node) => {
      if (checkIfAction(node)) {
        let castNode = node as BasicNodeModel;
        if (!(castNode.getName() in actionNodesData)) {
          saveActionNodeData(node);
          for (let key in node.getPorts()) {
            if (key !== 'parent') {
              if (node.getPorts()[key] instanceof InputPortModel) {
                castNode.addInputPort(key);
                actionNodesData[castNode.getName()]['input'] = actionNodesData[castNode.getName()]['input'].concat([key]);
              } else if (node.getPorts()[key] instanceof OutputPortModel) {
                castNode.addOutputPort(key);
                actionNodesData[castNode.getName()]['output'] = actionNodesData[castNode.getName()]['output'].concat([key]);
              }
            }
          }
        }
      }
    });
  }, [graphJson]);

  useEffect(() => {
    if (!isEditActionModalOpen && currentActionNode !== null) {
      for (const nodesId of actionNodesData[currentActionNode!.getName()]['ids']) {
        let genericActionNode = model.current.getNode(nodesId);
        let actionNode = genericActionNode as BasicNodeModel;
        for (const inputs of actionNodesData[actionNode.getName()]['input']) {
          if (!(inputs in actionNode.getPorts())) {
            actionNode.addInputPort(inputs);
          }
        }
        for (const outputs of actionNodesData[actionNode.getName()]['output']) {
          if (!(outputs in actionNode.getPorts())) {
            actionNode.addOutputPort(outputs);
          }
        }
        actionNode.setColor(actionNodesData[actionNode.getName()]['color'])
      }
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.current.serialize()));
    }
  }, [isEditActionModalOpen]);

  // Set the model in the engine ONLY on project change
  if (!forceNotReset.current) {
    if (graphJson === null) {
      const root_node = new BasicNodeModel('Tree Root', 'rgb(0,204,0)');
      root_node.setPosition(200, 200);
      root_node.addChildrenPort("Children Port");
      model.current.addAll(root_node);
    } else {
      try {
        // Try parsing the JSON string
        model.current.deserializeModel(graphJson, engine);

        // After deserialization, attach listeners to each node
        const nodes = model.current.getNodes();  // Assuming getNodes() method exists to retrieve all nodes
        nodes.forEach((node) => {
          attachPositionListener(node);
          attachLinkListener(node);
          attachClickListener(node);
        });
      } catch (e) {
        // Log the error for debugging
        console.error("An error occurred while parsing the JSON:", e);
      }
    }

    engine.setModel(model.current);
  }

  // Save action node data
  const saveActionNodeData = (node:any) => {
    if (actionNodesData[node.name]) {
      actionNodesData[node.name]['ids'] = actionNodesData[node.name]['ids'].concat([node.getID()]);
      for (const inputs of actionNodesData[node.name]['input']) {
          node.addInputPort(inputs);
      }
      for (const outputs of actionNodesData[node.name]['output']) {
          node.addOutputPort(outputs);
      }
      node.setColor(actionNodesData[node.getName()]['color'])
    } else {
      actionNodesData[node.name] = {input: [], output: [], ids: [node.getID()], color: node.getColor()};
    }
  }

  // Add the nodes default ports
  const addDefaultPorts = (node:any) => {

    console.log("Adding default ports");

    var nodeName = node.getName();
    if (nodeName === "RetryUntilSuccessful") node.addInputPort("num_attempts");
    else if (nodeName === "Repeat") node.addInputPort("num_cycles");
    else if (nodeName === "Delay") node.addInputPort("delay_ms");
  }

  const nodeTypeSelector = (nodeName:any) => {

    if (["Input port value", "Output port value"].includes(nodeName)) addTagNode(nodeName);
    else addBasicNode(nodeName);
  }

  // Function to add a new node
  const addBasicNode = (nodeName:any) => {

    // Control parameters
    let nodeColor = 'rgb(255,153,51)'; // Default color
    let hasInputPort = true;
    let hasOutputPort = true;
    let isAction = false;

    // Sequences
    if (["Sequence", "ReactiveSequence", "SequenceWithMemory"].includes(nodeName)) {
      nodeColor = 'rgb(0,128,255)';
    }
    // Fallbacks
    else if (["Fallback", "ReactiveFallback"].includes(nodeName)) {
      nodeColor = 'rgb(255,0,0)';
    }
    // Decorators
    else if (["RetryUntilSuccessful", "Inverter", "ForceSuccess", "ForceFailure", "KeepRunningUntilFailure", "Repeat", "RunOnce", "Delay"].includes(nodeName)) {
      nodeColor = 'rgb(255,153,51)';
    }
    // Actions
    else {
      nodeColor = 'rgb(128,0,128)';
      hasOutputPort = false;
      isAction = true;
    }

    // Create node
    const newNode = new BasicNodeModel(nodeName, nodeColor);

    if (isAction) {    
      saveActionNodeData(newNode);
      console.log(actionNodesData)
    }

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    attachLinkListener(newNode);
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
      console.log(model.current.getNodes());
      setProjectChanges(true);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.current.serialize()));
    }
  };

  const addTagNode = (nodeName:any) => {

    const newNode = new TagNodeModel('value', 'rgb(255,153,51)'); 
    
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
      setProjectChanges(true);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.current.serialize()));
    }
  }

  const deleteLastClickedNode = () => {
    if (model.current && lastClickedNodeId.current) {
      const node: any = model.current.getNode(lastClickedNodeId.current);
      if (node) {
        actionNodesData[node!.getName()]['ids'] = actionNodesData[node!.getName()]['ids'].filter(obj => obj !== lastClickedNodeId.current);
        node.remove();
        setProjectChanges(true);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      }
      lastClickedNodeId.current = "";
    }
  };

  const checkIfAction = (node:any) => {
    
    var name = node.name;

    if (node.getOptions().type === 'tag') {
      return false;
    }

    // Check if the node is a user written action
    return !(["Sequence", "ReactiveSequence", "SequenceWithMemory", 
        "Fallback", "ReactiveFallback", "RetryUntilSuccessful", "Inverter", "ForceSuccess", 
        "ForceFailure", "KeepRunningUntilFailure", "Repeat", "RunOnce", "Delay",
        "Input port value", "Output port value", "Tree Root"].includes(name))
  }

  const handleOpenEditActionModal = () => {
    if (lastClickedNodeId.current !== "") {
      console.log(model.current.getNodes());
      const genericNode = model.current.getNode(lastClickedNodeId.current);
      const node = genericNode as BasicNodeModel;
      console.log(node);
      if (checkIfAction(node)) {
        forceNotReset.current = true;
        node.deselectNode();
        setCurrentActionNode(node);
        setEditActionModalOpen(true);
      }
    }

  };

  const handleCloseEditActionModal = (color:any) => {
    setEditActionModalOpen(false);
    actionNodesData[currentActionNode.getName()]['color'] = 'rgb('+Math.round(color.rgb['r'])+','+Math.round(color.rgb['g'])+','+Math.round(color.rgb['b'])+')';
    console.log(actionNodesData)
  };

  const addInputPort = () => {
    if (currentActionNode) {
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        const portName = prompt("Enter the name for the new input port:");
        if (portName !== null) { // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.addInputPort(portName);
          console.log(currentActionNode)
          actionNodesData[currentActionNode.getName()]['input'] = actionNodesData[currentActionNode.getName()]['input'].concat([portName]);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[currentActionNode.getName()]['ids']) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              actionNode.addInputPort(portName);
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        window.alert("Ports can only be added to action nodes")
      }
    }
  };

  const addOutputPort = () => {
    if (currentActionNode) {
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        const portName = prompt("Enter the name for the new output port:");
        if (portName !== null) { // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.addOutputPort(portName);
          console.log(currentActionNode)
          actionNodesData[currentActionNode.getName()]['output'] = actionNodesData[currentActionNode.getName()]['output'].concat([portName]);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[currentActionNode.getName()]['ids']) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              actionNode.addOutputPort(portName);
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        window.alert("Ports can only be added to action nodes")
      }
    }
  };

  const deleteInputPort = (port:InputPortModel, portName:string) => {
    if (currentActionNode) {
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        if (port !== null) { // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.removeInputPort(port);
          actionNodesData[currentActionNode.getName()]['input'] = actionNodesData[currentActionNode.getName()]['input'].filter(item => item !== portName);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[currentActionNode.getName()]['ids']) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              actionNode.removeInputPort(port);
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        window.alert("Ports can only be removed from action nodes")
      }
    }
  }

  const deleteOutputPort = (port:OutputPortModel, portName:string) => {
    if (currentActionNode) {
      // Check restrictions
      if (checkIfAction(currentActionNode)) {
        // Now you can call your custom method
        if (port !== null) { // Check that the user didn't cancel
          setProjectChanges(true);
          currentActionNode.removeOutputPort(port);
          actionNodesData[currentActionNode.getName()]['output'] = actionNodesData[currentActionNode.getName()]['output'].filter(item => item !== portName);
          // Add the new port to all the cloned actions
          for (const nodesId of actionNodesData[currentActionNode.getName()]['ids']) {
            if (nodesId !== lastClickedNodeId.current) {
              let genericActionNode = model.current.getNode(nodesId);
              let actionNode = genericActionNode as BasicNodeModel;
              actionNode.removeOutputPort(port);
            }
          }
        }
        setCurrentActionNode(currentActionNode);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.current.serialize()));
      } else {
        window.alert("Ports can only be removed from action nodes")
      }
    }
  }

  const generateApp = () => {

    if (model.current) {
      const str = JSON.stringify(model.current.serialize());
    
      fetch("/tree_api/generate_app/", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ app_name: currentProjectname, content: str }),
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
        const a = document.createElement('a');
        a.style.display = 'none';
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
    if(gazeboEnabled) 
    {
      if(!appRunning) 
      {
        const tree_graph = JSON.stringify(model.current.serialize());
        fetch("/tree_api/get_simplified_app/", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ app_name: currentProjectname, content: tree_graph }),
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
          reader.onloadend = function() {
            var base64data = reader.result;                
            console.log(base64data);
            
            // Send the zip
            manager
            .run({type: "bt-studio", code: base64data})
            .then(() => {
              console.log("App running!");
              setAppRunning(true);
            })
            .catch((response:any) => {
              let linterMessage = JSON.stringify(response.data.message).split("\\n");
              alert(`Received linter message: ${linterMessage}`);
            });
          }
        })
        .catch((error) => {
          console.error("Error:", error);
        });
      }
      else {
        manager
        .terminate_application()
        .then(() => {
          console.log("App terminated!");
          setAppRunning(false);
        })
      }
    }
    else {
      console.log("Gazebo is not up!");
    }
  }
  
  return (
    <div>
      <NodeHeader 
        onNodeTypeSelected={nodeTypeSelector} 
        onDeleteNode={deleteLastClickedNode}
        onAddInputPort={handleOpenEditActionModal}
        onAddOutputPort={addOutputPort}
        onGenerateApp={generateApp}
        onRunApp={runApp}
        currentProjectname={currentProjectname}
      />
      <CanvasWidget className="canvas" engine={engine} />
      <EditActionModal
        isOpen={isEditActionModalOpen}
        onClose={handleCloseEditActionModal}
        currentActionNode={currentActionNode}
        addInputPort={addInputPort}
        addOutputPort={addOutputPort}
        deleteInputPort={deleteInputPort}
        deleteOutputPort={deleteOutputPort}
      />
    </div>
  );
};

export default React.memo(DiagramEditor);
