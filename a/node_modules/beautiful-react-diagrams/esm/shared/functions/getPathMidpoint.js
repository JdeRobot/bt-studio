/* beautiful-react-diagrams version: 0.5.1 */
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

export default getPathMidpoint;
