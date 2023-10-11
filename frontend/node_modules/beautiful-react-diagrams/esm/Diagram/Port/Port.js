/* beautiful-react-diagrams version: 0.5.1 */
import React, { useEffect } from 'react';
import { objectWithoutProperties as _objectWithoutProperties, extends as _extends } from '../../_virtual/_rollupPluginBabelHelpers.js';
import PropTypes from 'prop-types';
import useDrag from '../../shared/internal_hooks/useDrag.js';
import useCanvas from '../../shared/internal_hooks/useCanvas.js';
import getRelativePoint from '../../shared/functions/getRelativePoint.js';

var Port = function Port(props) {
  var id = props.id,
      canLink = props.canLink,
      alignment = props.alignment,
      onDragNewSegment = props.onDragNewSegment,
      onSegmentFail = props.onSegmentFail,
      onSegmentConnect = props.onSegmentConnect,
      onMount = props.onMount,
      type = props.type,
      rest = _objectWithoutProperties(props, ["id", "canLink", "alignment", "onDragNewSegment", "onSegmentFail", "onSegmentConnect", "onMount", "type"]);

  var canvas = useCanvas();

  var _useDrag = useDrag(),
      ref = _useDrag.ref,
      onDrag = _useDrag.onDrag,
      onDragEnd = _useDrag.onDragEnd;

  onDrag(function (event, info) {
    if (onDragNewSegment) {
      event.stopImmediatePropagation();
      event.stopPropagation();
      var from = getRelativePoint(info.start, [canvas.x, canvas.y]);
      var to = getRelativePoint([event.clientX, event.clientY], [canvas.x, canvas.y]);
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
  useEffect(function () {
    if (ref.current && onMount) {
      onMount(id, ref.current);
    }
  }, [ref.current]);
  return React.createElement("div", _extends({
    className: "bi bi-diagram-port",
    "data-port-id": id,
    ref: ref
  }, rest));
};

Port.propTypes = {
  id: PropTypes.oneOfType([PropTypes.string, PropTypes.symbol]).isRequired,
  type: PropTypes.oneOf(['input', 'output']).isRequired,
  onDragNewSegment: PropTypes.func,
  onSegmentFail: PropTypes.func,
  onSegmentConnect: PropTypes.func,
  canLink: PropTypes.func,
  onMount: PropTypes.func,
  alignment: PropTypes.oneOf(['right', 'left', 'top', 'bottom'])
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
var Port$1 = React.memo(Port);

export default Port$1;
