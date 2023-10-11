/**
 * Base factory for all the different types of entities.
 * Gets registered with the engine, and is used to generate models
 */
export class AbstractFactory {
    constructor(type) {
        this.type = type;
    }
    setDiagramEngine(engine) {
        this.engine = engine;
    }
    setFactoryBank(bank) {
        this.bank = bank;
    }
    getType() {
        return this.type;
    }
}
//# sourceMappingURL=AbstractFactory.js.map