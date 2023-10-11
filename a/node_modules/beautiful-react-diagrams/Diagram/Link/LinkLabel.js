/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var PropTypes = require('prop-types');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);
var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);

var LinkLabel = function LinkLabel(_ref) {
  var label = _ref.label,
      position = _ref.position;
  return React__default['default'].createElement("foreignObject", {
    x: position[0],
    y: position[1]
  }, React__default['default'].createElement("div", {
    className: "bi-diagram-link-label"
  }, label));
};

LinkLabel.propTypes = {
  label: PropTypes__default['default'].string.isRequired,
  position: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number).isRequired
};
var LinkLabel$1 = React__default['default'].memo(LinkLabel);

exports.default = LinkLabel$1;
