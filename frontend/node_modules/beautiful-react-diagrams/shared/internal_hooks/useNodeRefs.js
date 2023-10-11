/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var DiagramContext = require('../../Context/DiagramContext.js');

var useNodeRefs = function useNodeRefs() {
  var _useContext = React.useContext(DiagramContext['default']),
      nodes = _useContext.nodes;

  return nodes;
};

exports.default = useNodeRefs;
