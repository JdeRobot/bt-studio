/* beautiful-react-diagrams version: 0.5.1 */
import { useContext, useCallback, useEffect } from 'react';
import DiagramContext from '../../Context/DiagramContext.js';

var usePortRegistration = function usePortRegistration(inputs, outputs, onPortRegister) {
  var _useContext = useContext(DiagramContext),
      canvas = _useContext.canvas,
      ports = _useContext.ports;

  return useCallback(function (portId, portElement) {
    if (canvas && (inputs || outputs)) {
      if (ports && !ports[portId]) {
        onPortRegister(portId, portElement);
      }
    }
  }, [!!canvas, !!ports, inputs, outputs]);
};
var useNodeRegistration = function useNodeRegistration(ref, onNodeRegister, id) {
  var _useContext2 = useContext(DiagramContext),
      canvas = _useContext2.canvas,
      nodes = _useContext2.nodes;

  useEffect(function () {
    if (onNodeRegister && ref.current && canvas && nodes && !nodes[id]) {
      onNodeRegister(id, ref.current);
    }
  }, [ref.current, onNodeRegister, !!canvas, !!nodes, id]);
};

export { useNodeRegistration, usePortRegistration };
