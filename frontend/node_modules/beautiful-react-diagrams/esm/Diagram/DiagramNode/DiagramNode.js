/* beautiful-react-diagrams version: 0.5.1 */
import React, { useRef, useMemo } from 'react';
import { defineProperty as _defineProperty } from '../../_virtual/_rollupPluginBabelHelpers.js';
import PropTypes from 'prop-types';
import classNames from 'classnames';
import { PortType } from '../../shared/Types.js';
import getDiagramNodeStyle from './getDiagramNodeStyle.js';
import { usePortRegistration, useNodeRegistration } from '../../shared/internal_hooks/useContextRegistration.js';
import useDrag from '../../shared/internal_hooks/useDrag.js';
import portGenerator from './portGenerator.js';
import useNodeUnregistration from '../../shared/internal_hooks/useNodeUnregistration.js';

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
  var registerPort = usePortRegistration(inputs, outputs, onPortRegister);

  var _useDrag = useDrag({
    throttleBy: 14
  }),
      ref = _useDrag.ref,
      onDragStart = _useDrag.onDragStart,
      onDrag = _useDrag.onDrag;

  var dragStartPoint = useRef(coordinates);

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

  useNodeUnregistration(onNodeRemove, inputs, outputs, id);
  useNodeRegistration(ref, onMount, id);
  var classList = useMemo(function () {
    return classNames('bi bi-diagram-node', _defineProperty({}, "bi-diagram-node-".concat(type), !!type && !render), className);
  }, [type, className]);
  var options = {
    registerPort: registerPort,
    onDragNewSegment: onDragNewSegment,
    onSegmentFail: onSegmentFail,
    onSegmentConnect: onSegmentConnect
  };
  var InputPorts = inputs.map(portGenerator(options, 'input'));
  var OutputPorts = outputs.map(portGenerator(options, 'output'));
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
  return React.createElement("div", {
    className: classList,
    ref: ref,
    style: getDiagramNodeStyle(coordinates, disableDrag)
  }, render && typeof render === 'function' && render(customRenderProps), !render && React.createElement(React.Fragment, null, content, React.createElement("div", {
    className: "bi-port-wrapper"
  }, React.createElement("div", {
    className: "bi-input-ports"
  }, InputPorts), React.createElement("div", {
    className: "bi-output-ports"
  }, OutputPorts))));
};

DiagramNode.propTypes = {
  id: PropTypes.oneOfType([PropTypes.string]).isRequired,
  coordinates: PropTypes.arrayOf(PropTypes.number).isRequired,
  content: PropTypes.oneOfType([PropTypes.elementType, PropTypes.node]),
  inputs: PropTypes.arrayOf(PortType),
  outputs: PropTypes.arrayOf(PortType),
  type: PropTypes.oneOf(['default']),
  data: PropTypes.shape({}),
  render: PropTypes.func,
  onPositionChange: PropTypes.func,
  onMount: PropTypes.func,
  onPortRegister: PropTypes.func,
  onNodeRemove: PropTypes.func,
  onDragNewSegment: PropTypes.func,
  onSegmentFail: PropTypes.func,
  onSegmentConnect: PropTypes.func,
  className: PropTypes.string,
  disableDrag: PropTypes.bool
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
var DiagramNode$1 = React.memo(DiagramNode);

export default DiagramNode$1;
