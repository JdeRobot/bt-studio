/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var CURVE_FACTOR = 60;

var roundPoint = function roundPoint(point) {
  return [Math.floor(point[0]), Math.floor(point[1])];
};

var getXOffset = function getXOffset(alignment) {
  if (!alignment || alignment !== 'left' && alignment !== 'right') return 0;
  return alignment === 'left' ? -CURVE_FACTOR : CURVE_FACTOR;
};

var getYOffset = function getYOffset(alignment) {
  if (!alignment || alignment !== 'top' && alignment !== 'bottom') return 0;
  return alignment === 'top' ? CURVE_FACTOR : -CURVE_FACTOR;
};

var makeSvgPath = function makeSvgPath(startPoint, endPoint) {
  var options = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {
    type: 'curve'
  };
  if (!startPoint || !endPoint) return '';
  var roundedStart = roundPoint(startPoint);
  var roundedEnd = roundPoint(endPoint);
  var start = "".concat(roundedStart[0], ", ").concat(roundedStart[1]);
  var end = "".concat(roundedEnd[0], ", ").concat(roundedEnd[1]);

  if (options.type === 'bezier' && (options.inputAlignment || options.outputAlignment)) {
    var startControl = end;
    var endControl = start;

    if (options.inputAlignment) {
      var offsetX = roundedStart[0] + getXOffset(options.inputAlignment);
      var offsetY = roundedStart[1] + getYOffset(options.inputAlignment);
      endControl = "".concat(offsetX, ", ").concat(offsetY);
    }

    if (options.outputAlignment) {
      var _offsetX = roundedEnd[0] + getXOffset(options.outputAlignment);

      var _offsetY = roundedEnd[1] + getYOffset(options.outputAlignment);

      startControl = "".concat(_offsetX, ", ").concat(_offsetY);
    }

    return "M ".concat(start, " C ").concat(endControl, " ").concat(startControl, ", ").concat(end);
  }

  var ctrl = "".concat(roundedEnd[0], ", ").concat(roundedStart[1]);
  return "M ".concat(start, " Q ").concat(ctrl, ", ").concat(end);
};

exports.default = makeSvgPath;
