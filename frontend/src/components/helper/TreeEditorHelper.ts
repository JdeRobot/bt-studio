import { DiagramEngine, DiagramModel, LinkModel, NodeModel } from "@projectstorm/react-diagrams";

import { BasicNodeModel } from "../tree_editor/nodes/basic_node/BasicNodeModel";
import { TagNodeModel } from "../tree_editor/nodes/tag_node/TagNodeModel";
import { OutputPortModel } from "../tree_editor/nodes/basic_node/ports/output_port/OutputPortModel";
import { InputPortModel } from "../tree_editor/nodes/basic_node/ports/input_port/InputPortModel";

export enum ActionNodePortType {
  Input = 0,
  Output = 1,
}

const isActionNode = (nodeName: string) => {

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
  ].includes(nodeName);
};

export const addPort = (portName: string, node: BasicNodeModel, type: ActionNodePortType, engine:DiagramEngine, model:DiagramModel, diagramEditedCallback: React.Dispatch<React.SetStateAction<boolean>>, updateJsonState: Function) => {
  //TODO: type should be an enum
  // Check that the user didn't cancel
  if (!node || !portName) {
    return;
  }

  if (type === ActionNodePortType.Input) {
    node.addInputPort(portName);
  } else {
    node.addOutputPort(portName);
  }

  model.getNodes().forEach((oldNode: NodeModel) => {
    var convNode;
    var name;
    if (oldNode.getOptions().type === "tag") {
      return;
    }

    convNode = oldNode as BasicNodeModel;
    name = convNode.getName();
    if (isActionNode(name)) {
      if (name === node.getName() && node !== convNode) {
        if (type === ActionNodePortType.Input) {
          convNode.addInputPort(portName);
        } else {
          convNode.addOutputPort(portName);
        }
      }
    }
  });

  diagramEditedCallback(true);
  updateJsonState();
  engine.repaintCanvas();
};

const deletePortLink = (model: DiagramModel, portName: string, node: BasicNodeModel) => {
  // var link: LinkModel | undefined;
  var link: any;
  const nodePort = node.getPort(portName);

  if (nodePort) {
    link = Object.values(nodePort.links)[0];
    if (link) {
      model.removeLink(link);
    }
  }
};

export const removePort = (port: OutputPortModel | InputPortModel, node: BasicNodeModel, engine:DiagramEngine, model:DiagramModel, diagramEditedCallback: React.Dispatch<React.SetStateAction<boolean>>, updateJsonState: Function) => {
  //TODO: type should be an enum
  // Check that the user didn't cancel
  if (!node || !port) {
    return;
  }

  const portName = port.getName();

  deletePortLink(model, portName, node);

  if (port instanceof InputPortModel) {
    node.removeInputPort(port);
  } else {
    node.removeOutputPort(port);
  }

  model.getNodes().forEach((oldNode: NodeModel) => {
    var convNode;
    var name;
    if (oldNode.getOptions().type === "tag") {
      return;
    }

    convNode = oldNode as BasicNodeModel;
    name = convNode.getName();
    if (!isActionNode(name)) {
      return;
    }

    if (name === node.getName() && node.getID() !== convNode.getID()) {
      deletePortLink(model, portName, convNode);

      if (port instanceof InputPortModel) {
        convNode.removeInputPort(port);
      } else {
        convNode.removeOutputPort(port);
      }
    }
  });

  diagramEditedCallback(true);
  updateJsonState();
  engine.repaintCanvas();
};

export const changeColorNode = (rgb: [number, number, number], node: BasicNodeModel, engine:DiagramEngine, model:DiagramModel, diagramEditedCallback: React.Dispatch<React.SetStateAction<boolean>>, updateJsonState: Function) => {
  node.setColor(
    "rgb(" +
      Math.round(rgb[0]) +
      "," +
      Math.round(rgb[1]) +
      "," +
      Math.round(rgb[2]) +
      ")",
  );

  model.getNodes().forEach((oldNode: NodeModel) => {
    var convNode;
    var name;
    if (oldNode.getOptions().type === "tag") {
      return;
    }

    convNode = oldNode as BasicNodeModel;
    name = convNode.getName();
    if (!isActionNode(name)) {
      return;
    }

    if (name === node.getName() && node.getID() !== convNode.getID()) {
      convNode.setColor(node.getColor());
    }
  });

  diagramEditedCallback(true);
  updateJsonState();
  engine.repaintCanvas();
};