/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var getNodePortsId = require('../../shared/functions/getNodePortsId.js');
var findIndex = require('lodash.findindex');
var actionTypes = require('./actionTypes.js');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var findIndex__default = /*#__PURE__*/_interopDefaultLegacy(findIndex);

var schemaReducer = function schemaReducer(state, action) {
  switch (action.type) {
    case actionTypes.ON_CHANGE:
      return {
        nodes: action.payload.nodes || state.nodes || [],
        links: action.payload.links || state.links || []
      };

    case actionTypes.ON_NODE_ADD:
      if (state.nodes) {
        state.nodes.push(action.payload.node);
      }

      return {
        nodes: state.nodes || [],
        links: state.links || []
      };

    case actionTypes.ON_NODE_REMOVE:
      {
        var nextLinks = state.links || [];

        if (state.nodes) {
          var index = findIndex__default['default'](state.nodes, ['id', action.payload.nodeId]);
          var inputPorts = getNodePortsId['default'](state.nodes[index], 'inputs');
          var outputPorts = getNodePortsId['default'](state.nodes[index], 'outputs');
          nextLinks = nextLinks.filter(function (link) {
            return !inputPorts.includes(link.input) && !outputPorts.includes(link.output);
          });
          state.nodes.splice(index, 1);
        }

        return {
          nodes: state.nodes || [],
          links: nextLinks
        };
      }

    case actionTypes.ON_CONNECT:
      if (state.links) {
        state.links.push(action.payload.link);
      }

      return {
        nodes: state.nodes || [],
        links: state.links || []
      };

    default:
      return state;
  }
};

exports.default = schemaReducer;
