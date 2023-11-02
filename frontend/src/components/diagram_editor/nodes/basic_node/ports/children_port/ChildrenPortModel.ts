import { LinkModel, PortModel, DefaultLinkModel, DefaultPortModel } from '@projectstorm/react-diagrams';

export class ChildrenPortModel extends DefaultPortModel {
	constructor() {
		super({
			type: 'children',
			name: 'children',
			in: false
		});
	}

	createLinkModel(): LinkModel {
		return new DefaultLinkModel();
	}
}