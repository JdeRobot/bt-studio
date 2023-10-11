/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var DiagramContext = require('../../Context/DiagramContext.js');
var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var PropTypes = require('prop-types');
var beautifulReactHooks = require('beautiful-react-hooks');
var isEqual = require('lodash.isequal');
var classNames = require('classnames');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);
var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);
var isEqual__default = /*#__PURE__*/_interopDefaultLegacy(isEqual);
var classNames__default = /*#__PURE__*/_interopDefaultLegacy(classNames);

var DiagramCanvas = function DiagramCanvas(props) {
  var children = props.children,
      portRefs = props.portRefs,
      nodeRefs = props.nodeRefs,
      className = props.className,
      rest = _rollupPluginBabelHelpers.objectWithoutProperties(props, ["children", "portRefs", "nodeRefs", "className"]);

  var _useState = React.useState(null),
      _useState2 = _rollupPluginBabelHelpers.slicedToArray(_useState, 2),
      bbox = _useState2[0],
      setBoundingBox = _useState2[1];

  var canvasRef = React.useRef();
  var classList = classNames__default['default']('bi bi-diagram', className);

  var calculateBBox = function calculateBBox(el) {
    if (el) {
      var nextBBox = el.getBoundingClientRect();

      if (!isEqual__default['default'](nextBBox, bbox)) {
        setBoundingBox(nextBBox);
      }
    }
  };

  React.useEffect(function () {
    return calculateBBox(canvasRef.current);
  }, [canvasRef.current]);
  beautifulReactHooks.useWindowScroll(function () {
    return calculateBBox(canvasRef.current);
  });
  beautifulReactHooks.useWindowResize(function () {
    return calculateBBox(canvasRef.current);
  });
  return React__default['default'].createElement("div", _rollupPluginBabelHelpers['extends']({
    className: classList,
    ref: canvasRef
  }, rest), React__default['default'].createElement("div", {
    className: "bi-diagram-canvas"
  }, React__default['default'].createElement(DiagramContext['default'].Provider, {
    value: {
      canvas: bbox,
      ports: portRefs,
      nodes: nodeRefs,
      _nodes: {}
    }
  }, children)));
};

DiagramCanvas.propTypes = {
  portRefs: PropTypes__default['default'].shape({}),
  nodeRefs: PropTypes__default['default'].shape({}),
  className: PropTypes__default['default'].string
};
DiagramCanvas.defaultProps = {
  portRefs: {},
  nodeRefs: {},
  className: ''
};
var DiagramCanvas$1 = React__default['default'].memo(DiagramCanvas);

exports.default = DiagramCanvas$1;
