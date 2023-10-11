/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var isEqual = require('lodash.isequal');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e : { 'default': e }; }

var isEqual__default = /*#__PURE__*/_interopDefaultLegacy(isEqual);

var removeLinkFromArray = function removeLinkFromArray(link, links) {
  return links.filter(function (item) {
    return !isEqual__default['default'](item, link);
  });
};

exports.default = removeLinkFromArray;
