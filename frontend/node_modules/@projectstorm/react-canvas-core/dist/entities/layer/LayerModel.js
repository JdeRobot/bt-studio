import { BaseModel } from '../../core-models/BaseModel';
import _flatMap from 'lodash/flatMap';
import _forEach from 'lodash/forEach';
import _mapValues from 'lodash/mapValues';
export class LayerModel extends BaseModel {
    constructor(options = {}) {
        super(options);
        this.models = {};
        this.repaintEnabled = true;
    }
    deserialize(event) {
        super.deserialize(event);
        this.options.isSvg = !!event.data.isSvg;
        this.options.transformed = !!event.data.transformed;
        _forEach(event.data.models, (model) => {
            const modelOb = this.getChildModelFactoryBank(event.engine).getFactory(model.type).generateModel({
                initialConfig: model
            });
            modelOb.deserialize(Object.assign(Object.assign({}, event), { data: model }));
            this.addModel(modelOb);
        });
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { isSvg: this.options.isSvg, transformed: this.options.transformed, models: _mapValues(this.models, (model) => {
                return model.serialize();
            }) });
    }
    isRepaintEnabled() {
        return this.repaintEnabled;
    }
    allowRepaint(allow = true) {
        this.repaintEnabled = allow;
    }
    remove() {
        if (this.parent) {
            this.parent.removeLayer(this);
        }
        super.remove();
    }
    addModel(model) {
        model.setParent(this);
        this.models[model.getID()] = model;
    }
    getSelectionEntities() {
        return _flatMap(this.models, (model) => {
            return model.getSelectionEntities();
        });
    }
    getModels() {
        return this.models;
    }
    getModel(id) {
        return this.models[id];
    }
    removeModel(id) {
        const _id = typeof id === 'string' ? id : id.getID();
        if (this.models[_id]) {
            delete this.models[_id];
            return true;
        }
        return false;
    }
}
//# sourceMappingURL=LayerModel.js.map