/* beautiful-react-diagrams version: 0.5.1 */
import React from 'react';
import { extends as _extends } from '../../_virtual/_rollupPluginBabelHelpers.js';
import Port from '../Port/Port.js';

var portGenerator = function portGenerator(_ref, type) {
  var registerPort = _ref.registerPort,
      onDragNewSegment = _ref.onDragNewSegment,
      onSegmentFail = _ref.onSegmentFail,
      onSegmentConnect = _ref.onSegmentConnect;
  return function (port) {
    return React.createElement(Port, _extends({}, port, {
      onMount: registerPort,
      onDragNewSegment: onDragNewSegment,
      onSegmentFail: onSegmentFail,
      onSegmentConnect: onSegmentConnect,
      type: type,
      key: port.id
    }));
  };
};

export default portGenerator;
