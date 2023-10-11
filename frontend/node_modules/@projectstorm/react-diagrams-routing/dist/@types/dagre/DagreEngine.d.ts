import { DiagramModel } from '@projectstorm/react-diagrams-core';
import { GraphLabel } from 'dagre';
export interface DagreEngineOptions {
    graph?: GraphLabel;
    /**
     * Will also re-layout links
     */
    includeLinks?: boolean;
    nodeMargin?: number;
}
export declare class DagreEngine {
    options: DagreEngineOptions;
    constructor(options?: DagreEngineOptions);
    redistribute(model: DiagramModel): void;
    /**
     * TODO cleanup this method into smaller methods
     */
    refreshLinks(diagram: DiagramModel): void;
}
