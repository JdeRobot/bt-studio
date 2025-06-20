import { BaseObserver } from './BaseObserver';
import _values from 'lodash/values';
/**
 * Store and managed Factories that extend from Abstractfactory
 */
export class FactoryBank extends BaseObserver {
    constructor() {
        super();
        this.factories = {};
    }
    getFactories() {
        return _values(this.factories);
    }
    clearFactories() {
        for (let factory in this.factories) {
            this.deregisterFactory(factory);
        }
    }
    getFactory(type) {
        if (!this.factories[type]) {
            throw new Error(`Cannot find factory with type [${type}]`);
        }
        return this.factories[type];
    }
    registerFactory(factory) {
        factory.setFactoryBank(this);
        this.factories[factory.getType()] = factory;
        // todo fixme
        this.fireEvent({ factory }, 'factoryAdded');
    }
    deregisterFactory(type) {
        const factory = this.factories[type];
        factory.setFactoryBank(null);
        delete this.factories[type];
        // todo fixme
        this.fireEvent({ factory }, 'factoryRemoved');
    }
}
//# sourceMappingURL=FactoryBank.js.map