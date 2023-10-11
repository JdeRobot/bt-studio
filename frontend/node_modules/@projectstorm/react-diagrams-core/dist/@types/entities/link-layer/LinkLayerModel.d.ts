import { LayerModel, LayerModelGenerics } from '@projectstorm/react-canvas-core';
import { LinkModel } from '../link/LinkModel';
import { DiagramEngine } from '../../DiagramEngine';
export interface LinkLayerModelGenerics extends LayerModelGenerics {
    CHILDREN: LinkModel;
    ENGINE: DiagramEngine;
}
export declare class LinkLayerModel<G extends LinkLayerModelGenerics = LinkLayerModelGenerics> extends LayerModel<G> {
    constructor();
    addModel(model: G['CHILDREN']): void;
    getLinks(): {
        [id: string]: G["CHILDREN"];
    };
    getChildModelFactoryBank(engine: G['ENGINE']): import("@projectstorm/react-canvas-core").FactoryBank<import("@projectstorm/react-canvas-core").AbstractReactFactory<LinkModel<import("../link/LinkModel").LinkModelGenerics>, DiagramEngine>, import("@projectstorm/react-canvas-core").FactoryBankListener<import("@projectstorm/react-canvas-core").AbstractReactFactory<LinkModel<import("../link/LinkModel").LinkModelGenerics>, DiagramEngine>>>;
}
