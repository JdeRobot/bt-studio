/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var DiagramContext = require('../../Context/DiagramContext.js');

var usePortRegistration = function usePortRegistration(inputs, outputs, onPortRegister) {
  var _useContext = React.useContext(DiagramContext['default']),
      canvas = _useContext.canvas,
      ports = _useContext.ports;

  return React.useCallback(function (portId, portElement) {
    if (canvas && (inputs || outputs)) {
      if (ports && !ports[portId]) {
        onPortRegister(portId, portElement);
      }
    }
  }, [!!canvas, !!ports, inputs, outputs]);
};
var useNodeRegistration = function useNodeRegistration(ref, onNodeRegister, id) {
  var _useContext2 = React.useContext(DiagramContext['default']),
      canvas = _useContext2.canvas,
      nodes = _useContext2.nodes;

  React.useEffect(function () {
    if (onNodeRegister && ref.current && canvas && nodes && !nodes[id]) {
      onNodeRegister(id, ref.current);
    }
  }, [ref.current, onNodeRegister, !!canvas, !!nodes, id]);
};

exports.useNodeRegistration = useNodeRegistration;
exports.usePortRegistration = usePortRegistration;
