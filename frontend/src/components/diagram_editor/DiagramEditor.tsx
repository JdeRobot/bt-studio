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
  const [currentActionNode, setCurrentActionNode] = useState("");
  const [modelProjectName, setModelProjectName] = useState("");
  const [actionNodesData, setActionNodesData] = useState<{ [id: string]: ActionData; }>({});

  // Initial node position
  let lastMovedNodePosition = { x: 200, y: 200 };
  
  // Initialize state for last moved node ID
  let lastClickedNodeId = "";
  let lastClickedNodeName = "";
  const forceNotReset = useRef(false);

  // Store action nodes and their inputs and outputs
  interface ActionData {
    input: string[];
    output: string[];
    ids: string[];
  }
  // let actionNodesData: { [id: string]: ActionData; } = {};

  // Listeners
  const attachPositionListener = (node:any) => {
    node.registerListener({
      positionChanged: (event:any) => {
        lastMovedNodePosition = event.entity.getPosition();
        setProjectChanges(true);
        setModelJson(JSON.stringify(model.serialize())); // Serialize and update model JSON
      },
    });
  };

  const attachLinkListener = (node:any) => {
    node.registerListener({
      linksUpdated: (event:any) => {
        if (event.isCreated) {
          setProjectChanges(true);
          setModelJson(JSON.stringify(model.serialize())); // Update when a new link is created
        }
      },
    });
  };

  const attachClickListener = (node:any) => {
    node.registerListener({
      selectionChanged: (event:any) => {
        if (event.isSelected) {
          lastClickedNodeId = node.getID();
          node.selectNode();
          lastClickedNodeName = node.getName();
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
    const nodes = model.getNodes();  // Assuming getNodes() method exists to retrieve all nodes
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
    setCurrentActionNode(lastClickedNodeName);
    console.log(currentActionNode);
    if (!isEditActionModalOpen && currentActionNode !== "") {
      for (const nodesId of actionNodesData[currentActionNode]['ids']) {
        let genericActionNode = model.getNode(nodesId);
        let actionNode = genericActionNode as BasicNodeModel;
        console.log(model.getNodes());
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
      }
      forceNotReset.current = false;
    }
  }, [isEditActionModalOpen]);
  
  // Create the model
  var model = new DiagramModel();

  if (graphJson === null) {
    const root_node = new BasicNodeModel('Tree Root', 'rgb(0,204,0)');
    root_node.setPosition(200, 200);
    root_node.addChildrenPort("Children Port");
    model.addAll(root_node);
  } else {
    try {
      // Try parsing the JSON string
      model.deserializeModel(graphJson, engine);

      // After deserialization, attach listeners to each node
      const nodes = model.getNodes();  // Assuming getNodes() method exists to retrieve all nodes
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

  // Set the model in the engine ONLY on project change
  console.log("aaaa");
  if (!forceNotReset.current) {
    // return;
    engine.setModel(model);
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
    } else {
      actionNodesData[node.name] = {input: [], output: [], ids: [node.getID()]};
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
    }

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    attachLinkListener(newNode);
    lastClickedNodeId = newNode.getID();

    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;

    // Add ports
    if (hasInputPort) newNode.addParentPort("Parent Port");
    if (hasOutputPort) newNode.addChildrenPort("Children Port");
    addDefaultPorts(newNode);

    // Add the node to the model
    if (model) {
      model.addNode(newNode);
      console.log(model.getNodes());
      setProjectChanges(true);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.serialize()));
    }
  };

  const addTagNode = (nodeName:any) => {

    const newNode = new TagNodeModel('value', 'rgb(255,153,51)'); 
    
    if (nodeName === "Input port value") newNode.addOutputPort();
    else newNode.addInputPort();

    // Attach listener to this node
    attachPositionListener(newNode);
    attachClickListener(newNode);
    lastClickedNodeId = newNode.getID();
    
    // Setup the node position and ports
    var new_y = lastMovedNodePosition.y + 100;
    newNode.setPosition(lastMovedNodePosition.x, new_y);
    lastMovedNodePosition.y = new_y;

    // Add the node to the model
    if (model) {
      model.addNode(newNode);
      setProjectChanges(true);
      engine.repaintCanvas();
      setModelJson(JSON.stringify(model.serialize()));
    }
  }

  const deleteLastClickedNode = () => {
    if (model && lastClickedNodeId) {
      const node = model.getNode(lastClickedNodeId);
      if (node) {
        node.remove();
        setProjectChanges(true);
        engine.repaintCanvas();
        setModelJson(JSON.stringify(model.serialize()));
      }
      lastClickedNodeId = "";
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
    // if (lastClickedNodeId !== "") {
    //   console.log(model.getNodes());
    //   const genericNode = model.getNode(lastClickedNodeId);
    //   const node = genericNode as BasicNodeModel;
    //   console.log(node);
    //   if (checkIfAction(node)) {
    //     // (document.getElementById('actionNameEditor') as HTMLInputElement).value = node.getName();
    //   }
    // }
    // setProjectChanges(false);
    // setModelJson(JSON.stringify(model.serialize()));

    // axios.post('/tree_api/save_project/', {
    //   project_name: currentProjectname,
    //   graph_json: JSON.stringify(model.serialize())
    // })
    // .then(response => {
    //   if (response.data.success) {
    //     console.log('Project saved successfully.');
    //   } else {
    //     console.error('Error saving project:', response.data.message || 'Unknown error');
    //   }
    // })
    // .catch(error => {
    //   console.error('Axios Error:', error);
    // });
    forceNotReset.current = true;
    setEditActionModalOpen(true);
  };

  const handleCloseEditActionModal = () => {
    setEditActionModalOpen(false);
  };

  const addInputPort = () => {

    // if (model && lastClickedNodeId) {
      console.log(model.getNodes())

      const genericNode = model.getNode(lastClickedNodeId);
      console.log(currentActionNode)
      if (genericNode) {

        // Cast the node to BasicNodeModel
        const node = genericNode as BasicNodeModel;

        // Check restrictions
        if (checkIfAction(node)) {
          // Now you can call your custom method
          const portName = prompt("Enter the name for the new input port:");
          if (portName !== null) { // Check that the user didn't cancel
            setProjectChanges(true);
            node.addInputPort(portName);
            console.log(node)
            setCurrentActionNode(node.getName());
            actionNodesData[node.getName()]['input'] = actionNodesData[node.getName()]['input'].concat([portName]);
            // Add the new port to all the cloned actions
            for (const nodesId of actionNodesData[node.getName()]['ids']) {
              if (nodesId !== lastClickedNodeId) {
                let genericActionNode = model.getNode(nodesId);
                let actionNode = genericActionNode as BasicNodeModel;
                actionNode.addInputPort(portName);
              }
            }
          }
          engine.repaintCanvas();
          setModelJson(JSON.stringify(model.serialize()));
        } else {
          window.alert("Ports can only be added to action nodes")
        }
      }
    // }
  };

  const addOutputPort = () => {

    if (model && lastClickedNodeId) {
      
      const genericNode = model.getNode(lastClickedNodeId);
      if (genericNode) {

        // Cast the node to BasicNodeModel
        const node = genericNode as BasicNodeModel;
        
        // Check restrictions
        if (checkIfAction(node)) {
          // Now you can call your custom method
          const portName = prompt("Enter the name for the new output port:");
          if (portName !== null) { // Check that the user didn't cancel
            setProjectChanges(true);
            node.addOutputPort(portName);
            actionNodesData[node.getName()]['output'] = actionNodesData[node.getName()]['output'].concat([portName]);
            // Add the new port to all the cloned actions
            for (const nodesId of actionNodesData[node.getName()]['ids']) {
              if (nodesId !== lastClickedNodeId) {
                let genericActionNode = model.getNode(nodesId);
                let actionNode = genericActionNode as BasicNodeModel;
                actionNode.addOutputPort(portName);
              }
            }
          }
          engine.repaintCanvas();
          setModelJson(JSON.stringify(model.serialize()));
        } else {
          window.alert("Ports can only be added to action nodes")
        }
      }
    }
  };

  const generateApp = () => {

    if (model) {
      const str = JSON.stringify(model.serialize());
    
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
        const tree_graph = JSON.stringify(model.serialize());
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
        orig_engine={engine}
      />
    </div>
  );
};

export default React.memo(DiagramEditor);
