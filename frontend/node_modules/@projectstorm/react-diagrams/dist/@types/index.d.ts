import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import { CanvasEngineOptions } from '@projectstorm/react-canvas-core';
export * from '@projectstorm/react-canvas-core';
export * from '@projectstorm/react-diagrams-core';
export * from '@projectstorm/react-diagrams-defaults';
export * from '@projectstorm/react-diagrams-routing';
/**
 * Construct an engine with the defaults installed
 */
declare const _default: (options?: CanvasEngineOptions) => DiagramEngine;
export default _default;
