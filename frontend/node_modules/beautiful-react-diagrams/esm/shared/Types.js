/* beautiful-react-diagrams version: 0.5.1 */
import PropTypes from 'prop-types';

var LinkType = PropTypes.shape({
  input: PropTypes.string.isRequired,
  output: PropTypes.string.isRequired,
  label: PropTypes.node,
  readonly: PropTypes.bool,
  className: PropTypes.string
});
var PortAlignment = PropTypes.oneOf(['right', 'left', 'top', 'bottom']);
var PortType = PropTypes.shape({
  id: PropTypes.string.isRequired,
  canLink: PropTypes.func,
  alignment: PortAlignment
});
var NodeType = PropTypes.shape({
  id: PropTypes.string.isRequired,
  coordinates: PropTypes.arrayOf(PropTypes.number).isRequired,
  content: PropTypes.oneOfType([PropTypes.elementType, PropTypes.node]),
  inputs: PropTypes.arrayOf(PortType),
  outputs: PropTypes.arrayOf(PortType),
  type: PropTypes.oneOf(['default']),
  render: PropTypes.elementType,
  className: PropTypes.string
});
var SchemaType = PropTypes.shape({
  nodes: PropTypes.arrayOf(NodeType).isRequired,
  links: PropTypes.arrayOf(LinkType)
});

export { LinkType, NodeType, PortAlignment, PortType, SchemaType };
