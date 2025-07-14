import {
  LinkModel,
  DefaultLinkModel,
  DefaultPortModel,
} from "@projectstorm/react-diagrams";

export class InputPortModel extends DefaultPortModel {
  constructor(name: string) {
    super({
      type: "input",
      name: name,
      label: "input",
      in: true,
    });
  }

  createLinkModel(): LinkModel {
    return new DefaultLinkModel();
  }
}
