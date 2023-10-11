/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var getNodePortsId = function getNodePortsId(node, portType) {
  if (node[portType] && node[portType].length > 0) {
    return node[portType].map(function (port) {
      return port.id;
    });
  }

  return [];
};

exports.default = getNodePortsId;
