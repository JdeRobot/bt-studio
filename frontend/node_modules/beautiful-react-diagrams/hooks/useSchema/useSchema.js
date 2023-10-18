/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var actionTypes = require('./actionTypes.js');
var ensureNodeId = require('../../shared/functions/ensureNodeId.js');
var schemaReducer = require('./schemaReducer.js');

var initialState = {
  nodes: [],
  links: []
};

var useSchema = function useSchema() {
  var initialSchema = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : initialState;

  var _useReducer = React.useReducer(schemaReducer['default'], initialSchema),
      _useReducer2 = _rollupPluginBabelHelpers.slicedToArray(_useReducer, 2),
      schema = _useReducer2[0],
      dispatch = _useReducer2[1];

  var onChange = React.useCallback(function (_ref) {
    var nodes = _ref.nodes,
        links = _ref.links;
    return dispatch({
      type: actionTypes.ON_CHANGE,
      payload: {
        nodes: nodes,
        links: links
      }
    });
  }, []);
  var addNode = React.useCallback(function (node) {
    return dispatch({
      type: actionTypes.ON_NODE_ADD,
      payload: {
        node: ensureNodeId['default'](node)
      }
    });
  }, []);
  var removeNode = React.useCallback(function (node) {
    return dispatch({
      type: actionTypes.ON_NODE_REMOVE,
      payload: {
        nodeId: node.id
      }
    });
  }, []);
  var connect = React.useCallback(function (input, output) {
    return dispatch({
      type: actionTypes.ON_CONNECT,
      payload: {
        link: {
          input: input,
          output: output
        }
      }
    });
  }, []);
  return [schema, Object.freeze({
    onChange: onChange,
    addNode: addNode,
    removeNode: removeNode,
    connect: connect
  })];
};

exports.default = useSchema;
