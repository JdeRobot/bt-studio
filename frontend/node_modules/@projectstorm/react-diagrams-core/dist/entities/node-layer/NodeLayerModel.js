import { LayerModel } from '@projectstorm/react-canvas-core';
import { NodeModel } from '../node/NodeModel';
export class NodeLayerModel extends LayerModel {
    constructor() {
        super({
            type: 'diagram-nodes',
            isSvg: false,
            transformed: true
        });
    }
    addModel(model) {
        if (!(model instanceof NodeModel)) {
            throw new Error('Can only add nodes to this layer');
        }
        model.registerListener({
            entityRemoved: () => {
                this.getParent().removeNode(model);
            }
        });
        super.addModel(model);
    }
    getChildModelFactoryBank(engine) {
        return engine.getNodeFactories();
    }
    getNodes() {
        return this.getModels();
    }
}
//# sourceMappingURL=NodeLayerModel.js.map