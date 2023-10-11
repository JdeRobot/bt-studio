/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var PropTypes = require('prop-types');
var Types = require('../../shared/Types.js');
var Link = require('../Link/Link.js');
var Segment = require('../Segment/Segment.js');
var findInvolvedEntity = require('./findInvolvedEntity.js');
var removeLinkFromArray = require('./removeLinkFromArray.js');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);
var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);

var LinksCanvas = function LinksCanvas(props) {
  var nodes = props.nodes,
      segment = props.segment,
      onChange = props.onChange,
      links = props.links;
  var removeFromLinksArray = React.useCallback(function (link) {
    if (links.length > 0 && onChange) {
      var nextLinks = removeLinkFromArray['default'](link, links);
      onChange(nextLinks);
    }
  }, [links, onChange]);
  return React__default['default'].createElement("svg", {
    className: "bi bi-link-canvas-layer"
  }, links && links.length > 0 && links.map(function (link) {
    return React__default['default'].createElement(Link['default'], {
      link: link,
      input: findInvolvedEntity['default'](nodes, link.input),
      output: findInvolvedEntity['default'](nodes, link.output),
      onDelete: removeFromLinksArray,
      key: "".concat(link.input, "-").concat(link.output)
    });
  }), segment && React__default['default'].createElement(Segment['default'], segment));
};

LinksCanvas.propTypes = {
  nodes: PropTypes__default['default'].arrayOf(Types.NodeType),
  links: PropTypes__default['default'].arrayOf(Types.LinkType),
  segment: PropTypes__default['default'].exact({
    id: PropTypes__default['default'].string,
    from: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number),
    to: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number),
    alignment: Types.PortAlignment
  }),
  onChange: PropTypes__default['default'].func
};
LinksCanvas.defaultProps = {
  nodes: [],
  links: [],
  segment: undefined,
  onChange: undefined
};
var LinksCanvas$1 = React__default['default'].memo(LinksCanvas);

exports.default = LinksCanvas$1;
