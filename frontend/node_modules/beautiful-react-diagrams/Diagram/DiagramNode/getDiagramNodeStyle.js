/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var getDiagramNodeStyle = function getDiagramNodeStyle(coordinates, disableDrag) {
  return {
    left: coordinates[0],
    top: coordinates[1],
    cursor: disableDrag ? undefined : 'move'
  };
};

exports.default = getDiagramNodeStyle;
