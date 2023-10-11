import { LayerModel } from '../layer/LayerModel';
import { FactoryBank } from '../../core/FactoryBank';
import { AbstractModelFactory } from '../../core/AbstractModelFactory';
import { BaseModel } from '../../core-models/BaseModel';
import { SimpleClientRect } from '../../states/SelectionBoxState';
export declare class SelectionLayerModel extends LayerModel {
    box: SimpleClientRect;
    constructor();
    setBox(rect: SimpleClientRect): void;
    getChildModelFactoryBank(): FactoryBank<AbstractModelFactory<BaseModel>>;
}
