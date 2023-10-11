/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var ensureNodeId = function ensureNodeId(node) {
  node.id || (node.id = "node-".concat(Math.random().toString(36).substr(2, 9)));
  return node;
};

exports.default = ensureNodeId;
