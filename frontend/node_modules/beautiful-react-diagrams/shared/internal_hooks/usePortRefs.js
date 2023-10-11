/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var DiagramContext = require('../../Context/DiagramContext.js');

var usePortRefs = function usePortRefs() {
  var _useContext = React.useContext(DiagramContext['default']),
      ports = _useContext.ports;

  return ports;
};

exports.default = usePortRefs;
