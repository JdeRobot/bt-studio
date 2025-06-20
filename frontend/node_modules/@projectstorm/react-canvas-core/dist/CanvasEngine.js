import _debounce from 'lodash/debounce';
import { FactoryBank } from './core/FactoryBank';
import { BaseObserver } from './core/BaseObserver';
import { Point } from '@projectstorm/geometry';
import { ActionEventBus } from './core-actions/ActionEventBus';
import { PanAndZoomCanvasAction } from './actions/PanAndZoomCanvasAction';
import { ZoomCanvasAction } from './actions/ZoomCanvasAction';
import { DeleteItemsAction } from './actions/DeleteItemsAction';
import { StateMachine } from './core-state/StateMachine';
export class CanvasEngine extends BaseObserver {
    constructor(options = {}) {
        super();
        this.model = null;
        this.eventBus = new ActionEventBus(this);
        this.stateMachine = new StateMachine(this);
        this.layerFactories = new FactoryBank();
        this.registerFactoryBank(this.layerFactories);
        /**
         * Overrides the standard options with the possible given options
         */
        this.options = Object.assign({ registerDefaultDeleteItemsAction: true, registerDefaultZoomCanvasAction: true, repaintDebounceMs: 0 }, options);
        if (this.options.registerDefaultZoomCanvasAction === true) {
            this.eventBus.registerAction(new ZoomCanvasAction());
        }
        else if (this.options.registerDefaultPanAndZoomCanvasAction === true) {
            this.eventBus.registerAction(new PanAndZoomCanvasAction());
        }
        if (this.options.registerDefaultDeleteItemsAction === true) {
            this.eventBus.registerAction(new DeleteItemsAction());
        }
    }
    getStateMachine() {
        return this.stateMachine;
    }
    getRelativeMousePoint(event) {
        const point = this.getRelativePoint(event.clientX, event.clientY);
        return new Point((point.x - this.model.getOffsetX()) / (this.model.getZoomLevel() / 100.0), (point.y - this.model.getOffsetY()) / (this.model.getZoomLevel() / 100.0));
    }
    getRelativePoint(x, y) {
        const canvasRect = this.canvas.getBoundingClientRect();
        return new Point(x - canvasRect.left, y - canvasRect.top);
    }
    registerFactoryBank(factory) {
        factory.registerListener({
            factoryAdded: (event) => {
                event.factory.setDiagramEngine(this);
            },
            factoryRemoved: (event) => {
                event.factory.setDiagramEngine(null);
            }
        });
    }
    getActionEventBus() {
        return this.eventBus;
    }
    getLayerFactories() {
        return this.layerFactories;
    }
    getFactoryForLayer(layer) {
        if (typeof layer === 'string') {
            return this.layerFactories.getFactory(layer);
        }
        return this.layerFactories.getFactory(layer.getType());
    }
    setModel(model) {
        this.model = model;
        if (this.canvas) {
            requestAnimationFrame(() => {
                this.repaintCanvas();
            });
        }
    }
    getModel() {
        return this.model;
    }
    repaintCanvas(promise) {
        const { repaintDebounceMs } = this.options;
        /**
         * The actual repaint function
         */
        const repaint = () => {
            this.iterateListeners((listener) => {
                if (listener.repaintCanvas) {
                    listener.repaintCanvas();
                }
            });
        };
        // if the `repaintDebounceMs` option is > 0, then apply the debounce
        let repaintFn = repaint;
        if (repaintDebounceMs > 0) {
            repaintFn = _debounce(repaint, repaintDebounceMs);
        }
        if (promise) {
            return new Promise((resolve) => {
                const l = this.registerListener({
                    rendered: () => {
                        resolve();
                        l.deregister();
                    }
                });
                repaintFn();
            });
        }
        repaintFn();
    }
    setCanvas(canvas) {
        if (this.canvas !== canvas) {
            this.canvas = canvas;
            if (canvas) {
                this.fireEvent({}, 'canvasReady');
            }
        }
    }
    getCanvas() {
        return this.canvas;
    }
    getMouseElement(event) {
        return null;
    }
    zoomToFit() {
        const xFactor = this.canvas.clientWidth / this.canvas.scrollWidth;
        const yFactor = this.canvas.clientHeight / this.canvas.scrollHeight;
        const zoomFactor = xFactor < yFactor ? xFactor : yFactor;
        this.model.setZoomLevel(this.model.getZoomLevel() * zoomFactor);
        this.model.setOffset(0, 0);
        this.repaintCanvas();
    }
}
//# sourceMappingURL=CanvasEngine.js.map