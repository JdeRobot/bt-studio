import { DefaultPortModel } from './DefaultPortModel';
import { AbstractModelFactory } from '@projectstorm/react-canvas-core';
export class DefaultPortFactory extends AbstractModelFactory {
    constructor() {
        super('default');
    }
    generateModel() {
        return new DefaultPortModel({
            name: 'unknown'
        });
    }
}
//# sourceMappingURL=DefaultPortFactory.js.map