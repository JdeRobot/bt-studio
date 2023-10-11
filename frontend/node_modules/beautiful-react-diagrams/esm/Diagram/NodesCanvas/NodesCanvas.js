/* beautiful-react-diagrams version: 0.5.1 */
import React from 'react';
import { objectWithoutProperties as _objectWithoutProperties, extends as _extends } from '../../_virtual/_rollupPluginBabelHelpers.js';
import PropTypes from 'prop-types';
import { NodeType } from '../../shared/Types.js';
import DiagramNode from '../DiagramNode/DiagramNode.js';
import updateNodeCoordinates from './updateNodeCoordinates.js';

var NodesCanvas = function NodesCanvas(props) {
  var nodes = props.nodes,
      onPortRegister = props.onPortRegister,
      onNodeRegister = props.onNodeRegister,
      onNodeRemove = props.onNodeRemove,
      onDragNewSegment = props.onDragNewSegment,
      onSegmentFail = props.onSegmentFail,
      onSegmentConnect = props.onSegmentConnect,
      onChange = props.onChange;

  var onNodePositionChange = function onNodePositionChange(nodeId, newCoordinates) {
    if (onChange) {
      var nextNodes = updateNodeCoordinates(nodeId, newCoordinates, nodes);
      onChange(nextNodes);
    }
  };

  return nodes && nodes.length > 0 && nodes.map(function (_ref) {
    var data = _ref.data,
        node = _objectWithoutProperties(_ref, ["data"]);

    return React.createElement(DiagramNode, _extends({}, node, {
      data: data,
      onPositionChange: onNodePositionChange,
      onPortRegister: onPortRegister,
      onNodeRemove: onNodeRemove,
      onDragNewSegment: onDragNewSegment,
      onSegmentFail: onSegmentFail,
      onSegmentConnect: onSegmentConnect,
      onMount: onNodeRegister,
      key: node.id
    }));
  });
};

NodesCanvas.propTypes = {
  nodes: PropTypes.arrayOf(NodeType),
  onChange: PropTypes.func,
  onNodeRegister: PropTypes.func,
  onPortRegister: PropTypes.func,
  onNodeRemove: PropTypes.func,
  onDragNewSegment: PropTypes.func,
  onSegmentFail: PropTypes.func,
  onSegmentConnect: PropTypes.func
};
NodesCanvas.defaultProps = {
  nodes: [],
  onChange: undefined,
  onNodeRegister: undefined,
  onPortRegister: undefined,
  onNodeRemove: undefined,
  onDragNewSegment: undefined,
  onSegmentFail: undefined,
  onSegmentConnect: undefined
};
var NodesCanvas$1 = React.memo(NodesCanvas);

export default NodesCanvas$1;
