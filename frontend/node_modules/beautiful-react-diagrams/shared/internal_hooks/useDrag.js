/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var _rollupPluginBabelHelpers = require('../../_virtual/_rollupPluginBabelHelpers.js');
var throttle = require('lodash.throttle');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var throttle__default = /*#__PURE__*/_interopDefaultLegacy(throttle);

var defaultOptions = {
  ref: undefined,
  throttleBy: 0
};

var getEventCoordinates = function getEventCoordinates(event) {
  return [event.clientX, event.clientY];
};

var createCallbackRef = function createCallbackRef(ref) {
  return React.useCallback(function (callback) {
    if (!ref.current || callback !== ref.current) {
      ref.current = callback;
    }
  }, [ref.current]);
};

var useDrag = function useDrag() {
  var options = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : defaultOptions;
  var targetRef = options.ref || React.useRef();
  var dragStartHandlerRef = React.useRef();
  var dragHandlerRef = React.useRef();
  var dragEndHandlerRef = React.useRef();

  var _useRef = React.useRef({
    isDragging: false,
    start: null,
    end: null,
    offset: null
  }),
      info = _useRef.current;

  var onDragStart = React.useCallback(function (event) {
    if (!info.isDragging && targetRef.current.contains(event.target)) {
      info.isDragging = true;
      info.end = null;
      info.offset = null;
      info.start = getEventCoordinates(event);

      if (dragStartHandlerRef.current) {
        dragStartHandlerRef.current(event, _rollupPluginBabelHelpers.objectSpread2({}, info));
      }
    }
  }, [targetRef.current, info, dragStartHandlerRef.current]);
  var onDrag = React.useCallback(throttle__default['default'](function (event) {
    if (info.isDragging) {
      info.offset = [info.start[0] - event.clientX, info.start[1] - event.clientY];

      if (dragHandlerRef.current) {
        dragHandlerRef.current(event, _rollupPluginBabelHelpers.objectSpread2({}, info));
      }
    }
  }, options.throttleBy), [targetRef.current, info, dragHandlerRef.current]);
  var onDragEnd = React.useCallback(function (event) {
    if (info.isDragging) {
      info.isDragging = false;
      info.end = getEventCoordinates(event);

      if (dragEndHandlerRef.current) {
        dragEndHandlerRef.current(event, _rollupPluginBabelHelpers.objectSpread2({}, info));
      }
    }
  }, [targetRef.current, info, dragEndHandlerRef.current]);
  React.useEffect(function () {
    var _onDragStart = function _onDragStart(e) {
      return onDragStart(e);
    };

    var _onDrag = function _onDrag(e) {
      return onDrag(e);
    };

    var _onDragEnd = function _onDragEnd(e) {
      return onDragEnd(e);
    };

    if (targetRef.current) {
      targetRef.current.addEventListener('mousedown', _onDragStart);
      document.addEventListener('mousemove', _onDrag);
      document.addEventListener('mouseup', _onDragEnd);
    }

    return function () {
      if (targetRef.current) {
        targetRef.current.removeEventListener('mousedown', _onDragStart);
        document.removeEventListener('mousemove', _onDrag);
        document.removeEventListener('mouseup', _onDragEnd);
      }
    };
  }, [targetRef.current]);
  return {
    ref: targetRef,
    onDragStart: createCallbackRef(dragStartHandlerRef),
    onDrag: createCallbackRef(dragHandlerRef),
    onDragEnd: createCallbackRef(dragEndHandlerRef)
  };
};

exports.default = useDrag;
