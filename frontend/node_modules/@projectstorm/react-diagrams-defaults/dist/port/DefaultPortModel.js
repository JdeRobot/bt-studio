import { PortModel, PortModelAlignment } from '@projectstorm/react-diagrams-core';
import { DefaultLinkModel } from '../link/DefaultLinkModel';
export class DefaultPortModel extends PortModel {
    constructor(options, name, label) {
        if (!!name) {
            options = {
                in: !!options,
                name: name,
                label: label
            };
        }
        options = options;
        super(Object.assign({ label: options.label || options.name, alignment: options.in ? PortModelAlignment.LEFT : PortModelAlignment.RIGHT, type: 'default' }, options));
    }
    deserialize(event) {
        super.deserialize(event);
        this.options.in = event.data.in;
        this.options.label = event.data.label;
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { in: this.options.in, label: this.options.label });
    }
    link(port, factory) {
        let link = this.createLinkModel(factory);
        link.setSourcePort(this);
        link.setTargetPort(port);
        return link;
    }
    canLinkToPort(port) {
        if (port instanceof DefaultPortModel) {
            return this.options.in !== port.getOptions().in;
        }
        return true;
    }
    createLinkModel(factory) {
        let link = super.createLinkModel();
        if (!link && factory) {
            return factory.generateModel({});
        }
        return link || new DefaultLinkModel();
    }
}
//# sourceMappingURL=DefaultPortModel.js.map