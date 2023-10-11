/* beautiful-react-diagrams version: 0.5.1 */
import React, { useState, useRef, useEffect } from 'react';
import DiagramContext from '../../Context/DiagramContext.js';
import { objectWithoutProperties as _objectWithoutProperties, slicedToArray as _slicedToArray, extends as _extends } from '../../_virtual/_rollupPluginBabelHelpers.js';
import PropTypes from 'prop-types';
import { useWindowScroll, useWindowResize } from 'beautiful-react-hooks';
import isEqual from 'lodash.isequal';
import classNames from 'classnames';

var DiagramCanvas = function DiagramCanvas(props) {
  var children = props.children,
      portRefs = props.portRefs,
      nodeRefs = props.nodeRefs,
      className = props.className,
      rest = _objectWithoutProperties(props, ["children", "portRefs", "nodeRefs", "className"]);

  var _useState = useState(null),
      _useState2 = _slicedToArray(_useState, 2),
      bbox = _useState2[0],
      setBoundingBox = _useState2[1];

  var canvasRef = useRef();
  var classList = classNames('bi bi-diagram', className);

  var calculateBBox = function calculateBBox(el) {
    if (el) {
      var nextBBox = el.getBoundingClientRect();

      if (!isEqual(nextBBox, bbox)) {
        setBoundingBox(nextBBox);
      }
    }
  };

  useEffect(function () {
    return calculateBBox(canvasRef.current);
  }, [canvasRef.current]);
  useWindowScroll(function () {
    return calculateBBox(canvasRef.current);
  });
  useWindowResize(function () {
    return calculateBBox(canvasRef.current);
  });
  return React.createElement("div", _extends({
    className: classList,
    ref: canvasRef
  }, rest), React.createElement("div", {
    className: "bi-diagram-canvas"
  }, React.createElement(DiagramContext.Provider, {
    value: {
      canvas: bbox,
      ports: portRefs,
      nodes: nodeRefs,
      _nodes: {}
    }
  }, children)));
};

DiagramCanvas.propTypes = {
  portRefs: PropTypes.shape({}),
  nodeRefs: PropTypes.shape({}),
  className: PropTypes.string
};
DiagramCanvas.defaultProps = {
  portRefs: {},
  nodeRefs: {},
  className: ''
};
var DiagramCanvas$1 = React.memo(DiagramCanvas);

export default DiagramCanvas$1;
