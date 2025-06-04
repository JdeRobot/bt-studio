import {
  LinkModel,
  DefaultLinkModel,
  DefaultPortModel,
} from "@projectstorm/react-diagrams";

export class TagOutputPortModel extends DefaultPortModel {
  constructor() {
    super({
      type: "tag output",
      name: "tag output",
      label: "tag output",
      in: false,
    });
  }

  createLinkModel(): LinkModel {
    return new DefaultLinkModel();
  }
}
