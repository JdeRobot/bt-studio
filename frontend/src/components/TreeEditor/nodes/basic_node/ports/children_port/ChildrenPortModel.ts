import {
  LinkModel,
  DefaultLinkModel,
  DefaultPortModel,
} from "@projectstorm/react-diagrams";

export class ChildrenPortModel extends DefaultPortModel {
  constructor() {
    super({
      type: "children",
      name: "children",
      label: "children",
      in: false,
    });
  }

  createLinkModel(): LinkModel {
    return new DefaultLinkModel();
  }
}
