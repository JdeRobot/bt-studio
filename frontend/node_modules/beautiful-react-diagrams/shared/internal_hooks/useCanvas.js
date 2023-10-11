/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var DiagramContext = require('../../Context/DiagramContext.js');

var useCanvas = function useCanvas() {
  var _useContext = React.useContext(DiagramContext['default']),
      canvas = _useContext.canvas;

  return canvas;
};

exports.default = useCanvas;
