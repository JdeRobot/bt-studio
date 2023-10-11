/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var findIndex = require('lodash.findindex');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var findIndex__default = /*#__PURE__*/_interopDefaultLegacy(findIndex);

var updateNodeCoordinates = function updateNodeCoordinates(nodeId, coordinates, nodes) {
  var index = findIndex__default['default'](nodes, ['id', nodeId]);

  if (index > -1 && !nodes[index].disableDrag) {
    nodes[index].coordinates = coordinates;
  }

  return nodes;
};

exports.default = updateNodeCoordinates;
