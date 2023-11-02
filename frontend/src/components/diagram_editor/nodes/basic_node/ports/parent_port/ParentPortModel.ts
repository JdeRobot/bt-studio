import { LinkModel, DefaultPortModel, DefaultLinkModel } from '@projectstorm/react-diagrams';

export class ParentPortModel extends DefaultPortModel {
	constructor() {
		super({
			type: 'parent',
			name: 'parent',
			in: true
		});
	}

	createLinkModel(): LinkModel {
		return new DefaultLinkModel();
	}
}