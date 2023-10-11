/* beautiful-react-diagrams version: 0.5.1 */
import { useRef, useCallback, useEffect } from 'react';
import { objectSpread2 as _objectSpread2 } from '../../_virtual/_rollupPluginBabelHelpers.js';
import throttle from 'lodash.throttle';

var defaultOptions = {
  ref: undefined,
  throttleBy: 0
};

var getEventCoordinates = function getEventCoordinates(event) {
  return [event.clientX, event.clientY];
};

var createCallbackRef = function createCallbackRef(ref) {
  return useCallback(function (callback) {
    if (!ref.current || callback !== ref.current) {
      ref.current = callback;
    }
  }, [ref.current]);
};

var useDrag = function useDrag() {
  var options = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : defaultOptions;
  var targetRef = options.ref || useRef();
  var dragStartHandlerRef = useRef();
  var dragHandlerRef = useRef();
  var dragEndHandlerRef = useRef();

  var _useRef = useRef({
    isDragging: false,
    start: null,
    end: null,
    offset: null
  }),
      info = _useRef.current;

  var onDragStart = useCallback(function (event) {
    if (!info.isDragging && targetRef.current.contains(event.target)) {
      info.isDragging = true;
      info.end = null;
      info.offset = null;
      info.start = getEventCoordinates(event);

      if (dragStartHandlerRef.current) {
        dragStartHandlerRef.current(event, _objectSpread2({}, info));
      }
    }
  }, [targetRef.current, info, dragStartHandlerRef.current]);
  var onDrag = useCallback(throttle(function (event) {
    if (info.isDragging) {
      info.offset = [info.start[0] - event.clientX, info.start[1] - event.clientY];

      if (dragHandlerRef.current) {
        dragHandlerRef.current(event, _objectSpread2({}, info));
      }
    }
  }, options.throttleBy), [targetRef.current, info, dragHandlerRef.current]);
  var onDragEnd = useCallback(function (event) {
    if (info.isDragging) {
      info.isDragging = false;
      info.end = getEventCoordinates(event);

      if (dragEndHandlerRef.current) {
        dragEndHandlerRef.current(event, _objectSpread2({}, info));
      }
    }
  }, [targetRef.current, info, dragEndHandlerRef.current]);
  useEffect(function () {
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

export default useDrag;
