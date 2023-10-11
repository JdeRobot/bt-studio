import { NodeModel } from './entities/node/NodeModel';
import { boundingBoxFromPolygons, Rectangle } from '@projectstorm/geometry';
import { CanvasEngine, FactoryBank, Toolkit } from '@projectstorm/react-canvas-core';
/**
 * Passed as a parameter to the DiagramWidget
 */
export class DiagramEngine extends CanvasEngine {
    constructor(options = {}) {
        super(options);
        this.maxNumberPointsPerLink = 1000;
        // create banks for the different factory types
        this.nodeFactories = new FactoryBank();
        this.linkFactories = new FactoryBank();
        this.portFactories = new FactoryBank();
        this.labelFactories = new FactoryBank();
        const setup = (factory) => {
            factory.registerListener({
                factoryAdded: (event) => {
                    event.factory.setDiagramEngine(this);
                },
                factoryRemoved: (event) => {
                    event.factory.setDiagramEngine(null);
                }
            });
        };
        setup(this.nodeFactories);
        setup(this.linkFactories);
        setup(this.portFactories);
        setup(this.labelFactories);
    }
    /**
     * Gets a model and element under the mouse cursor
     */
    getMouseElement(event) {
        var target = event.target;
        var diagramModel = this.model;
        //is it a port
        var element = Toolkit.closest(target, '.port[data-name]');
        if (element) {
            var nodeElement = Toolkit.closest(target, '.node[data-nodeid]');
            return diagramModel.getNode(nodeElement.getAttribute('data-nodeid')).getPort(element.getAttribute('data-name'));
        }
        //look for a point
        element = Toolkit.closest(target, '.point[data-id]');
        if (element) {
            return diagramModel.getLink(element.getAttribute('data-linkid')).getPointModel(element.getAttribute('data-id'));
        }
        //look for a link
        element = Toolkit.closest(target, '[data-linkid]');
        if (element) {
            return diagramModel.getLink(element.getAttribute('data-linkid'));
        }
        //look for a node
        element = Toolkit.closest(target, '.node[data-nodeid]');
        if (element) {
            return diagramModel.getNode(element.getAttribute('data-nodeid'));
        }
        return null;
    }
    //!-------------- FACTORIES ------------
    getNodeFactories() {
        return this.nodeFactories;
    }
    getLinkFactories() {
        return this.linkFactories;
    }
    getLabelFactories() {
        return this.labelFactories;
    }
    getPortFactories() {
        return this.portFactories;
    }
    getFactoryForNode(node) {
        if (typeof node === 'string') {
            return this.nodeFactories.getFactory(node);
        }
        return this.nodeFactories.getFactory(node.getType());
    }
    getFactoryForLink(link) {
        if (typeof link === 'string') {
            return this.linkFactories.getFactory(link);
        }
        return this.linkFactories.getFactory(link.getType());
    }
    getFactoryForLabel(label) {
        if (typeof label === 'string') {
            return this.labelFactories.getFactory(label);
        }
        return this.labelFactories.getFactory(label.getType());
    }
    getFactoryForPort(port) {
        if (typeof port === 'string') {
            return this.portFactories.getFactory(port);
        }
        return this.portFactories.getFactory(port.getType());
    }
    generateWidgetForLink(link) {
        return this.getFactoryForLink(link).generateReactWidget({ model: link });
    }
    generateWidgetForNode(node) {
        return this.getFactoryForNode(node).generateReactWidget({ model: node });
    }
    getNodeElement(node) {
        const selector = this.canvas.querySelector(`.node[data-nodeid="${node.getID()}"]`);
        if (selector === null) {
            throw new Error('Cannot find Node element with nodeID: [' + node.getID() + ']');
        }
        return selector;
    }
    getNodePortElement(port) {
        var selector = this.canvas.querySelector(`.port[data-name="${port.getName()}"][data-nodeid="${port.getParent().getID()}"]`);
        if (selector === null) {
            throw new Error('Cannot find Node Port element with nodeID: [' +
                port.getParent().getID() +
                '] and name: [' +
                port.getName() +
                ']');
        }
        return selector;
    }
    getPortCenter(port) {
        return this.getPortCoords(port).getOrigin();
    }
    /**
     * Calculate rectangular coordinates of the port passed in.
     */
    getPortCoords(port, element) {
        if (!this.canvas) {
            throw new Error('Canvas needs to be set first');
        }
        if (!element) {
            element = this.getNodePortElement(port);
        }
        const sourceRect = element.getBoundingClientRect();
        const point = this.getRelativeMousePoint({
            clientX: sourceRect.left,
            clientY: sourceRect.top
        });
        const zoom = this.model.getZoomLevel() / 100.0;
        return Rectangle.fromPointAndSize(point, sourceRect.width / zoom, sourceRect.height / zoom);
    }
    /**
     * Determine the width and height of the node passed in.
     * It currently assumes nodes have a rectangular shape, can be overriden for customised shapes.
     */
    getNodeDimensions(node) {
        if (!this.canvas) {
            return {
                width: 0,
                height: 0
            };
        }
        const nodeElement = this.getNodeElement(node);
        const nodeRect = nodeElement.getBoundingClientRect();
        return {
            width: nodeRect.width,
            height: nodeRect.height
        };
    }
    getBoundingNodesRect(nodes) {
        if (nodes) {
            if (nodes.length === 0) {
                return new Rectangle();
            }
            return new Rectangle(boundingBoxFromPolygons(nodes.map((node) => node.getBoundingBox())));
        }
    }
    zoomToFitSelectedNodes(options) {
        const nodes = this.model
            .getSelectedEntities()
            .filter((entity) => entity instanceof NodeModel);
        this.zoomToFitNodes({
            margin: options.margin,
            maxZoom: options.maxZoom,
            nodes: nodes.length > 0 ? nodes : null
        });
    }
    zoomToFitNodes(options) {
        let margin = options || 0;
        let nodes = [];
        let maxZoom = null;
        if (!!options && typeof options == 'object') {
            margin = options.margin || 0;
            nodes = options.nodes || [];
            maxZoom = options.maxZoom || null;
        }
        // no node selected
        if (nodes.length === 0) {
            nodes = this.model.getNodes();
        }
        const nodesRect = this.getBoundingNodesRect(nodes);
        if (nodesRect) {
            // there is something we should zoom on
            let canvasRect = this.canvas.getBoundingClientRect();
            const calculate = (margin = 0) => {
                // work out zoom
                const xFactor = this.canvas.clientWidth / (nodesRect.getWidth() + margin * 2);
                const yFactor = this.canvas.clientHeight / (nodesRect.getHeight() + margin * 2);
                let zoomFactor = xFactor < yFactor ? xFactor : yFactor;
                if (maxZoom && zoomFactor > maxZoom) {
                    zoomFactor = maxZoom;
                }
                return {
                    zoom: zoomFactor,
                    x: canvasRect.width / 2 -
                        ((nodesRect.getWidth() + margin * 2) / 2 + nodesRect.getTopLeft().x) * zoomFactor +
                        margin,
                    y: canvasRect.height / 2 -
                        ((nodesRect.getHeight() + margin * 2) / 2 + nodesRect.getTopLeft().y) * zoomFactor +
                        margin
                };
            };
            let params = calculate(0);
            if (margin) {
                if (params.x < margin || params.y < margin) {
                    params = calculate(margin);
                }
            }
            // apply
            this.model.setZoomLevel(params.zoom * 100);
            this.model.setOffset(params.x, params.y);
            this.repaintCanvas();
        }
    }
    getMaxNumberPointsPerLink() {
        return this.maxNumberPointsPerLink;
    }
    setMaxNumberPointsPerLink(max) {
        this.maxNumberPointsPerLink = max;
    }
}
//# sourceMappingURL=DiagramEngine.js.map