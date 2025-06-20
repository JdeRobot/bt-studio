import { LayerModel } from '../layer/LayerModel';
export class SelectionLayerModel extends LayerModel {
    constructor() {
        super({
            transformed: false,
            isSvg: false,
            type: 'selection'
        });
    }
    setBox(rect) {
        this.box = rect;
    }
    getChildModelFactoryBank() {
        // is not used as it doesnt serialize
        return null;
    }
}
//# sourceMappingURL=SelectionLayerModel.js.map