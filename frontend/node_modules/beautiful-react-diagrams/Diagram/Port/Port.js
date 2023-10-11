/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var PropTypes = require('prop-types');
var useDrag = require('../../shared/internal_hooks/useDrag.js');
var useCanvas = require('../../shared/internal_hooks/useCanvas.js');
var getRelativePoint = require('../../shared/functions/getRelativePoint.js');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var React__default = /*#__PURE__*/_interopDefaultLegacy(React);
var PropTypes__default = /*#__PURE__*/_interopDefaultLegacy(PropTypes);

var Port = function Port(props) {
  var id = props.id,
      canLink = props.canLink,
      alignment = props.alignment,
      onDragNewSegment = props.onDragNewSegment,
      onSegmentFail = props.onSegmentFail,
      onSegmentConnect = props.onSegmentConnect,
      onMount = props.onMount,
      type = props.type,
      rest = _rollupPluginBabelHelpers.objectWithoutProperties(props, ["id", "canLink", "alignment", "onDragNewSegment", "onSegmentFail", "onSegmentConnect", "onMount", "type"]);

  var canvas = useCanvas['default']();

  var _useDrag = useDrag['default'](),
      ref = _useDrag.ref,
      onDrag = _useDrag.onDrag,
      onDragEnd = _useDrag.onDragEnd;

  onDrag(function (event, info) {
    if (onDragNewSegment) {
      event.stopImmediatePropagation();
      event.stopPropagation();
      var from = getRelativePoint['default'](info.start, [canvas.x, canvas.y]);
      var to = getRelativePoint['default']([event.clientX, event.clientY], [canvas.x, canvas.y]);
      onDragNewSegment(id, from, to, alignment);
    }
  });
  onDragEnd(function (event) {
    var targetPort = event.target.getAttribute('data-port-id');

    if (targetPort && event.target !== ref.current && canLink(id, targetPort, type) && onSegmentConnect) {
      var args = type === 'input' ? [id, targetPort, type] : [targetPort, id, type];
      onSegmentConnect.apply(void 0, args);
      return;
    }

    onSegmentFail && onSegmentFail(id, type);
  });
  React.useEffect(function () {
    if (ref.current && onMount) {
      onMount(id, ref.current);
    }
  }, [ref.current]);
  return React__default['default'].createElement("div", _rollupPluginBabelHelpers['extends']({
    className: "bi bi-diagram-port",
    "data-port-id": id,
    ref: ref
  }, rest));
};

Port.propTypes = {
  id: PropTypes__default['default'].oneOfType([PropTypes__default['default'].string, PropTypes__default['default'].symbol]).isRequired,
  type: PropTypes__default['default'].oneOf(['input', 'output']).isRequired,
  onDragNewSegment: PropTypes__default['default'].func,
  onSegmentFail: PropTypes__default['default'].func,
  onSegmentConnect: PropTypes__default['default'].func,
  canLink: PropTypes__default['default'].func,
  onMount: PropTypes__default['default'].func,
  alignment: PropTypes__default['default'].oneOf(['right', 'left', 'top', 'bottom'])
};
Port.defaultProps = {
  onDragNewSegment: undefined,
  onSegmentFail: undefined,
  onSegmentConnect: undefined,
  canLink: function canLink() {
    return true;
  },
  onMount: undefined,
  alignment: undefined
};
var Port$1 = React__default['default'].memo(Port);

exports.default = Port$1;
