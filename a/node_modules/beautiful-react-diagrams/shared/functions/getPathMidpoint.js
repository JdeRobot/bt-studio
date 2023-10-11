/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var getPathMidpoint = function getPathMidpoint(pathElement) {
  if (pathElement.getTotalLength && pathElement.getPointAtLength) {
    var midpoint = pathElement.getTotalLength() / 2;

    var _pathElement$getPoint = pathElement.getPointAtLength(midpoint),
        x = _pathElement$getPoint.x,
        y = _pathElement$getPoint.y;

    return [x, y];
  }

  return [0, 0];
};

exports.default = getPathMidpoint;
