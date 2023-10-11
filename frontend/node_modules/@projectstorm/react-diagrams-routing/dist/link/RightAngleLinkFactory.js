import * as React from 'react';
import { RightAngleLinkWidget } from './RightAngleLinkWidget';
import { DefaultLinkFactory } from '@projectstorm/react-diagrams-defaults';
import { RightAngleLinkModel } from './RightAngleLinkModel';
/**
 * @author Daniel Lazar
 */
export class RightAngleLinkFactory extends DefaultLinkFactory {
    constructor() {
        super(RightAngleLinkFactory.NAME);
    }
    generateModel(event) {
        return new RightAngleLinkModel();
    }
    generateReactWidget(event) {
        return React.createElement(RightAngleLinkWidget, { diagramEngine: this.engine, link: event.model, factory: this });
    }
}
RightAngleLinkFactory.NAME = 'rightAngle';
//# sourceMappingURL=RightAngleLinkFactory.js.map