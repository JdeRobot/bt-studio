/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var Port = require('../Port/Port.js');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);

var portGenerator = function portGenerator(_ref, type) {
  var registerPort = _ref.registerPort,
      onDragNewSegment = _ref.onDragNewSegment,
      onSegmentFail = _ref.onSegmentFail,
      onSegmentConnect = _ref.onSegmentConnect;
  return function (port) {
    return React__default['default'].createElement(Port['default'], _rollupPluginBabelHelpers['extends']({}, port, {
      onMount: registerPort,
      onDragNewSegment: onDragNewSegment,
      onSegmentFail: onSegmentFail,
      onSegmentConnect: onSegmentConnect,
      type: type,
      key: port.id
    }));
  };
};

exports.default = portGenerator;
