import { PathFindingLinkFactory } from './PathFindingLinkFactory';
import { DefaultLinkModel } from '@projectstorm/react-diagrams-defaults';
export class PathFindingLinkModel extends DefaultLinkModel {
    constructor(options = {}) {
        super(Object.assign({ type: PathFindingLinkFactory.NAME }, options));
    }
    performanceTune() {
        return false;
    }
}
//# sourceMappingURL=PathFindingLinkModel.js.map