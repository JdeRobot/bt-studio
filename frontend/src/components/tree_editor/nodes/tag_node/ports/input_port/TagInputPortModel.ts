import {
  LinkModel,
  DefaultLinkModel,
  DefaultPortModel,
} from "@projectstorm/react-diagrams";

export class TagInputPortModel extends DefaultPortModel {
  constructor() {
    super({
      type: "tag input",
      name: "tag input",
      label: "tag input",
      in: true,
    });
  }

  createLinkModel(): LinkModel {
    return new DefaultLinkModel();
  }
}
