/* beautiful-react-diagrams version: 0.5.1 */
import { useReducer, useCallback } from 'react';
import { slicedToArray as _slicedToArray } from '../../_virtual/_rollupPluginBabelHelpers.js';
import { ON_CHANGE, ON_NODE_ADD, ON_NODE_REMOVE, ON_CONNECT } from './actionTypes.js';
import ensureNodeId from '../../shared/functions/ensureNodeId.js';
import schemaReducer from './schemaReducer.js';

var initialState = {
  nodes: [],
  links: []
};

var useSchema = function useSchema() {
  var initialSchema = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : initialState;

  var _useReducer = useReducer(schemaReducer, initialSchema),
      _useReducer2 = _slicedToArray(_useReducer, 2),
      schema = _useReducer2[0],
      dispatch = _useReducer2[1];

  var onChange = useCallback(function (_ref) {
    var nodes = _ref.nodes,
        links = _ref.links;
    return dispatch({
      type: ON_CHANGE,
      payload: {
        nodes: nodes,
        links: links
      }
    });
  }, []);
  var addNode = useCallback(function (node) {
    return dispatch({
      type: ON_NODE_ADD,
      payload: {
        node: ensureNodeId(node)
      }
    });
  }, []);
  var removeNode = useCallback(function (node) {
    return dispatch({
      type: ON_NODE_REMOVE,
      payload: {
        nodeId: node.id
      }
    });
  }, []);
  var connect = useCallback(function (input, output) {
    return dispatch({
      type: ON_CONNECT,
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

export default useSchema;
