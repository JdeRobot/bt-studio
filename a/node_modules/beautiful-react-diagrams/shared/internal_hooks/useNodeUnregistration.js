/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var getNodePortsId = require('../functions/getNodePortsId.js');

var useNodeUnregistration = function useNodeUnregistration(onNodeRemove, inputs, outputs, id) {
  React.useEffect(function () {
    return function () {
      if (onNodeRemove) {
        var node = {
          inputs: inputs,
          outputs: outputs
        };
        var inputsPort = getNodePortsId['default'](node, 'inputs');
        var outputsPort = getNodePortsId['default'](node, 'outputs');
        onNodeRemove(id, inputsPort, outputsPort);
      }
    };
  }, []);
};

exports.default = useNodeUnregistration;
