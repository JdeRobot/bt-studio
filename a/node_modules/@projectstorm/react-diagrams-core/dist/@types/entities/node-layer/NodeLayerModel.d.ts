import { LayerModel, LayerModelGenerics } from '@projectstorm/react-canvas-core';
import { NodeModel } from '../node/NodeModel';
import { DiagramEngine } from '../../DiagramEngine';
export interface NodeLayerModelGenerics extends LayerModelGenerics {
    CHILDREN: NodeModel;
    ENGINE: DiagramEngine;
}
export declare class NodeLayerModel<G extends NodeLayerModelGenerics = NodeLayerModelGenerics> extends LayerModel<G> {
    constructor();
    addModel(model: G['CHILDREN']): void;
    getChildModelFactoryBank(engine: G['ENGINE']): import("@projectstorm/react-canvas-core").FactoryBank<import("@projectstorm/react-canvas-core").AbstractReactFactory<NodeModel<import("../node/NodeModel").NodeModelGenerics>, DiagramEngine>, import("@projectstorm/react-canvas-core").FactoryBankListener<import("@projectstorm/react-canvas-core").AbstractReactFactory<NodeModel<import("../node/NodeModel").NodeModelGenerics>, DiagramEngine>>>;
    getNodes(): {
        [id: string]: G["CHILDREN"];
    };
}
