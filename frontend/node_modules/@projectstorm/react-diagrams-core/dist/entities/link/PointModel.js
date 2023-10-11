import { BasePositionModel } from '@projectstorm/react-canvas-core';
export class PointModel extends BasePositionModel {
    constructor(options) {
        super(Object.assign(Object.assign({}, options), { type: 'point' }));
        this.parent = options.link;
    }
    isConnectedToPort() {
        return this.parent.getPortForPoint(this) !== null;
    }
    getLink() {
        return this.getParent();
    }
    remove() {
        //clear references
        if (this.parent) {
            this.parent.removePoint(this);
        }
        super.remove();
    }
    isLocked() {
        return super.isLocked() || this.getParent().isLocked();
    }
}
//# sourceMappingURL=PointModel.js.map