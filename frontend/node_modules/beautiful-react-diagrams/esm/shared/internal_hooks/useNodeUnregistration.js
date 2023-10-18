/* beautiful-react-diagrams version: 0.5.1 */
import { useEffect } from 'react';
import getNodePortsId from '../functions/getNodePortsId.js';

var useNodeUnregistration = function useNodeUnregistration(onNodeRemove, inputs, outputs, id) {
  useEffect(function () {
    return function () {
      if (onNodeRemove) {
        var node = {
          inputs: inputs,
          outputs: outputs
        };
        var inputsPort = getNodePortsId(node, 'inputs');
        var outputsPort = getNodePortsId(node, 'outputs');
        onNodeRemove(id, inputsPort, outputsPort);
      }
    };
  }, []);
};

export default useNodeUnregistration;
