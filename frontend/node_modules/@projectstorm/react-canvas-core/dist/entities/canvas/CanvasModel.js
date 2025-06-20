import _filter from 'lodash/filter';
import _flatMap from 'lodash/flatMap';
import _forEach from 'lodash/forEach';
import _map from 'lodash/map';
import _values from 'lodash/values';
import { BaseEntity } from '../../core-models/BaseEntity';
export class CanvasModel extends BaseEntity {
    constructor(options = {}) {
        super(Object.assign({ zoom: 100, gridSize: 0, offsetX: 0, offsetY: 0 }, options));
        this.layers = [];
    }
    getSelectionEntities() {
        return _flatMap(this.layers, (layer) => {
            return layer.getSelectionEntities();
        });
    }
    getSelectedEntities() {
        return _filter(this.getSelectionEntities(), (ob) => {
            return ob.isSelected();
        });
    }
    clearSelection() {
        _forEach(this.getSelectedEntities(), (element) => {
            element.setSelected(false);
        });
    }
    getModels() {
        return _flatMap(this.layers, (layer) => {
            return _values(layer.getModels());
        });
    }
    addLayer(layer) {
        layer.setParent(this);
        layer.registerListener({
            entityRemoved: (event) => { }
        });
        this.layers.push(layer);
    }
    removeLayer(layer) {
        const index = this.layers.indexOf(layer);
        if (index !== -1) {
            this.layers.splice(index, 1);
            return true;
        }
        return false;
    }
    getLayers() {
        return this.layers;
    }
    setGridSize(size = 0) {
        this.options.gridSize = size;
        this.fireEvent({ size: size }, 'gridUpdated');
    }
    getGridPosition(pos) {
        if (this.options.gridSize === 0) {
            return pos;
        }
        return this.options.gridSize * Math.floor((pos + this.options.gridSize / 2) / this.options.gridSize);
    }
    deserializeModel(data, engine) {
        const models = {};
        const promises = {};
        const resolvers = {};
        const event = {
            data: data,
            engine: engine,
            registerModel: (model) => {
                models[model.getID()] = model;
                if (resolvers[model.getID()]) {
                    resolvers[model.getID()](model);
                }
            },
            getModel(id) {
                if (models[id]) {
                    return Promise.resolve(models[id]);
                }
                if (!promises[id]) {
                    promises[id] = new Promise((resolve) => {
                        resolvers[id] = resolve;
                    });
                }
                return promises[id];
            }
        };
        this.deserialize(event);
    }
    deserialize(event) {
        super.deserialize(event);
        this.options.offsetX = event.data.offsetX;
        this.options.offsetY = event.data.offsetY;
        this.options.zoom = event.data.zoom;
        this.options.gridSize = event.data.gridSize;
        _forEach(event.data.layers, (layer) => {
            const layerOb = event.engine.getFactoryForLayer(layer.type).generateModel({
                initialConfig: layer
            });
            layerOb.deserialize(Object.assign(Object.assign({}, event), { data: layer }));
            this.addLayer(layerOb);
        });
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { offsetX: this.options.offsetX, offsetY: this.options.offsetY, zoom: this.options.zoom, gridSize: this.options.gridSize, layers: _map(this.layers, (layer) => {
                return layer.serialize();
            }) });
    }
    setZoomLevel(zoom) {
        this.options.zoom = zoom;
        this.fireEvent({ zoom }, 'zoomUpdated');
    }
    setOffset(offsetX, offsetY) {
        this.options.offsetX = offsetX;
        this.options.offsetY = offsetY;
        this.fireEvent({ offsetX, offsetY }, 'offsetUpdated');
    }
    setOffsetX(offsetX) {
        this.setOffset(offsetX, this.options.offsetY);
    }
    setOffsetY(offsetY) {
        this.setOffset(this.options.offsetX, offsetY);
    }
    getOffsetY() {
        return this.options.offsetY;
    }
    getOffsetX() {
        return this.options.offsetX;
    }
    getZoomLevel() {
        return this.options.zoom;
    }
}
//# sourceMappingURL=CanvasModel.js.map