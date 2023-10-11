/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var PropTypes = require('prop-types');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);

var LinkType = PropTypes__default['default'].shape({
  input: PropTypes__default['default'].string.isRequired,
  output: PropTypes__default['default'].string.isRequired,
  label: PropTypes__default['default'].node,
  readonly: PropTypes__default['default'].bool,
  className: PropTypes__default['default'].string
});
var PortAlignment = PropTypes__default['default'].oneOf(['right', 'left', 'top', 'bottom']);
var PortType = PropTypes__default['default'].shape({
  id: PropTypes__default['default'].string.isRequired,
  canLink: PropTypes__default['default'].func,
  alignment: PortAlignment
});
var NodeType = PropTypes__default['default'].shape({
  id: PropTypes__default['default'].string.isRequired,
  coordinates: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number).isRequired,
  content: PropTypes__default['default'].oneOfType([PropTypes__default['default'].elementType, PropTypes__default['default'].node]),
  inputs: PropTypes__default['default'].arrayOf(PortType),
  outputs: PropTypes__default['default'].arrayOf(PortType),
  type: PropTypes__default['default'].oneOf(['default']),
  render: PropTypes__default['default'].elementType,
  className: PropTypes__default['default'].string
});
var SchemaType = PropTypes__default['default'].shape({
  nodes: PropTypes__default['default'].arrayOf(NodeType).isRequired,
  links: PropTypes__default['default'].arrayOf(LinkType)
});

exports.LinkType = LinkType;
exports.NodeType = NodeType;
exports.PortAlignment = PortAlignment;
exports.PortType = PortType;
exports.SchemaType = SchemaType;
