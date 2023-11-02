import { LinkModel, PortModel, DefaultLinkModel, DefaultPortModel } from '@projectstorm/react-diagrams';

export class OutputPortModel extends DefaultPortModel {
	constructor(name: string) {
		super({
			type: 'output',
			name: name,
			label: 'output',
			in: false
		});
	}

	createLinkModel(): LinkModel {
		return new DefaultLinkModel();
	}
}