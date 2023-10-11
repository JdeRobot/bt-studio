/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var PropTypes = require('prop-types');
var Types = require('../../shared/Types.js');
var makeSvgPath = require('../../shared/functions/makeSvgPath.js');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);
var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);

var Segment = function Segment(props) {
  var from = props.from,
      to = props.to,
      alignment = props.alignment;
  var pathOptions = {
    type: 'bezier',
    inputAlignment: alignment
  };
  var path = React.useMemo(function () {
    return makeSvgPath['default'](from, to, pathOptions);
  }, [from, to, alignment]);
  return React__default['default'].createElement("g", {
    className: "bi-diagram-segment"
  }, React__default['default'].createElement("path", {
    d: path
  }), React__default['default'].createElement("circle", {
    r: "6.5",
    cx: to[0],
    cy: to[1]
  }));
};

Segment.propTypes = {
  from: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number).isRequired,
  to: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number).isRequired,
  alignment: Types.PortAlignment
};
Segment.defaultProps = {
  alignment: undefined
};
var Segment$1 = React__default['default'].memo(Segment);

exports.default = Segment$1;
