/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var ensureNodeId = require('./ensureNodeId.js');
var validators = require('./validators.js');

var createSchema = function createSchema(schema) {
  var next = _rollupPluginBabelHelpers.objectSpread2({}, schema);

  next.nodes || (next.nodes = []);
  next.links || (next.links = []);
  next.nodes.forEach(ensureNodeId['default']);
  validators.validateSchema(next);
  return next;
};

exports.default = createSchema;
