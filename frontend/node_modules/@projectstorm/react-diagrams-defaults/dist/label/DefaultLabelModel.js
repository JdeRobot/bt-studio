import { LabelModel } from '@projectstorm/react-diagrams-core';
export class DefaultLabelModel extends LabelModel {
    constructor(options = {}) {
        super(Object.assign({ offsetY: options.offsetY == null ? -23 : options.offsetY, type: 'default' }, options));
    }
    setLabel(label) {
        this.options.label = label;
    }
    deserialize(event) {
        super.deserialize(event);
        this.options.label = event.data.label;
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { label: this.options.label });
    }
}
//# sourceMappingURL=DefaultLabelModel.js.map