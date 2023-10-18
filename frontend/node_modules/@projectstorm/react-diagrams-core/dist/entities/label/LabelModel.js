import { BaseModel } from '@projectstorm/react-canvas-core';
export class LabelModel extends BaseModel {
    constructor(options) {
        super(Object.assign(Object.assign({}, options), { offsetX: options.offsetX || 0, offsetY: options.offsetY || 0 }));
    }
    deserialize(event) {
        super.deserialize(event);
        this.options.offsetX = event.data.offsetX;
        this.options.offsetY = event.data.offsetY;
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { offsetX: this.options.offsetX, offsetY: this.options.offsetY });
    }
}
//# sourceMappingURL=LabelModel.js.map