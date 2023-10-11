/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');

var findInvolvedEntity = function findInvolvedEntity(nodes, entityId) {
  var type = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 'node';
  if (!entityId || !nodes || nodes.length === 0) return undefined;
  var result;
  var index = 0;

  while (index < nodes.length && !result) {
    var node = nodes[index];

    if (node.id === entityId) {
      result = {
        type: type,
        entity: _rollupPluginBabelHelpers.objectSpread2({}, node)
      };
    } else {
      result = findInvolvedEntity(node.inputs, entityId, 'port') || findInvolvedEntity(node.outputs, entityId, 'port');
    }

    index += 1;
  }

  return result;
};

exports.default = findInvolvedEntity;
