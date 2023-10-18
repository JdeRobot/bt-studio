import { LayerModel } from '@projectstorm/react-canvas-core';
import { LinkModel } from '../link/LinkModel';
export class LinkLayerModel extends LayerModel {
    constructor() {
        super({
            type: 'diagram-links',
            isSvg: true,
            transformed: true
        });
    }
    addModel(model) {
        if (!(model instanceof LinkModel)) {
            throw new Error('Can only add links to this layer');
        }
        model.registerListener({
            entityRemoved: () => {
                this.getParent().removeLink(model);
            }
        });
        super.addModel(model);
    }
    getLinks() {
        return this.getModels();
    }
    getChildModelFactoryBank(engine) {
        return engine.getLinkFactories();
    }
}
//# sourceMappingURL=LinkLayerModel.js.map