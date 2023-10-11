/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var PropTypes = require('prop-types');
var classNames = require('classnames');
var Types = require('../../shared/Types.js');
var getDiagramNodeStyle = require('./getDiagramNodeStyle.js');
var useContextRegistration = require('../../shared/internal_hooks/useContextRegistration.js');
var useDrag = require('../../shared/internal_hooks/useDrag.js');
var portGenerator = require('./portGenerator.js');
var useNodeUnregistration = require('../../shared/internal_hooks/useNodeUnregistration.js');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);
var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);
var classNames__default = /*#__PURE__*/_interopDefaultLegacy(classNames);

var DiagramNode = function DiagramNode(props) {
  var id = props.id,
      content = props.content,
      coordinates = props.coordinates,
      type = props.type,
      inputs = props.inputs,
      outputs = props.outputs,
      data = props.data,
      onPositionChange = props.onPositionChange,
      onPortRegister = props.onPortRegister,
      onNodeRemove = props.onNodeRemove,
      onDragNewSegment = props.onDragNewSegment,
      onMount = props.onMount,
      onSegmentFail = props.onSegmentFail,
      onSegmentConnect = props.onSegmentConnect,
      render = props.render,
      className = props.className,
      disableDrag = props.disableDrag;
  var registerPort = useContextRegistration.usePortRegistration(inputs, outputs, onPortRegister);

  var _useDrag = useDrag['default']({
    throttleBy: 14
  }),
      ref = _useDrag.ref,
      onDragStart = _useDrag.onDragStart,
      onDrag = _useDrag.onDrag;

  var dragStartPoint = React.useRef(coordinates);

  if (!disableDrag) {
    onDragStart(function () {
      dragStartPoint.current = coordinates;
    });
    onDrag(function (event, info) {
      if (onPositionChange) {
        event.stopImmediatePropagation();
        event.stopPropagation();
        var nextCoords = [dragStartPoint.current[0] - info.offset[0], dragStartPoint.current[1] - info.offset[1]];
        onPositionChange(id, nextCoords);
      }
    });
  }

  useNodeUnregistration['default'](onNodeRemove, inputs, outputs, id);
  useContextRegistration.useNodeRegistration(ref, onMount, id);
  var classList = React.useMemo(function () {
    return classNames__default['default']('bi bi-diagram-node', _rollupPluginBabelHelpers.defineProperty({}, "bi-diagram-node-".concat(type), !!type && !render), className);
  }, [type, className]);
  var options = {
    registerPort: registerPort,
    onDragNewSegment: onDragNewSegment,
    onSegmentFail: onSegmentFail,
    onSegmentConnect: onSegmentConnect
  };
  var InputPorts = inputs.map(portGenerator['default'](options, 'input'));
  var OutputPorts = outputs.map(portGenerator['default'](options, 'output'));
  var customRenderProps = {
    id: id,
    render: render,
    content: content,
    type: type,
    inputs: InputPorts,
    outputs: OutputPorts,
    data: data,
    className: className
  };
  return React__default['default'].createElement("div", {
    className: classList,
    ref: ref,
    style: getDiagramNodeStyle['default'](coordinates, disableDrag)
  }, render && typeof render === 'function' && render(customRenderProps), !render && React__default['default'].createElement(React__default['default'].Fragment, null, content, React__default['default'].createElement("div", {
    className: "bi-port-wrapper"
  }, React__default['default'].createElement("div", {
    className: "bi-input-ports"
  }, InputPorts), React__default['default'].createElement("div", {
    className: "bi-output-ports"
  }, OutputPorts))));
};

DiagramNode.propTypes = {
  id: PropTypes__default['default'].oneOfType([PropTypes__default['default'].string]).isRequired,
  coordinates: PropTypes__default['default'].arrayOf(PropTypes__default['default'].number).isRequired,
  content: PropTypes__default['default'].oneOfType([PropTypes__default['default'].elementType, PropTypes__default['default'].node]),
  inputs: PropTypes__default['default'].arrayOf(Types.PortType),
  outputs: PropTypes__default['default'].arrayOf(Types.PortType),
  type: PropTypes__default['default'].oneOf(['default']),
  data: PropTypes__default['default'].shape({}),
  render: PropTypes__default['default'].func,
  onPositionChange: PropTypes__default['default'].func,
  onMount: PropTypes__default['default'].func,
  onPortRegister: PropTypes__default['default'].func,
  onNodeRemove: PropTypes__default['default'].func,
  onDragNewSegment: PropTypes__default['default'].func,
  onSegmentFail: PropTypes__default['default'].func,
  onSegmentConnect: PropTypes__default['default'].func,
  className: PropTypes__default['default'].string,
  disableDrag: PropTypes__default['default'].bool
};
DiagramNode.defaultProps = {
  type: 'default',
  content: '',
  inputs: [],
  outputs: [],
  data: {},
  onPositionChange: undefined,
  render: undefined,
  onMount: undefined,
  onPortRegister: undefined,
  onNodeRemove: undefined,
  onDragNewSegment: undefined,
  onSegmentFail: undefined,
  onSegmentConnect: undefined,
  className: '',
  disableDrag: false
};
var DiagramNode$1 = React__default['default'].memo(DiagramNode);

exports.default = DiagramNode$1;
