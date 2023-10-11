/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var _rollupPluginBabelHelpers = require('../_virtual/_rollupPluginBabelHelpers.js');

var DiagramSchemaError = function (_Error) {
  _rollupPluginBabelHelpers.inherits(DiagramSchemaError, _Error);

  var _super = _rollupPluginBabelHelpers.createSuper(DiagramSchemaError);

  function DiagramSchemaError() {
    var _this;

    _rollupPluginBabelHelpers.classCallCheck(this, DiagramSchemaError);

    _this = _super.call(this);
    _this.name = 'Diagram Schema Error';
    return _this;
  }

  return DiagramSchemaError;
}(_rollupPluginBabelHelpers.wrapNativeSuper(Error));
var ERR = Object.freeze({
  MUST_HAVE_NODES: function MUST_HAVE_NODES() {
    return new DiagramSchemaError('A valid schema should have the required property \'nodes\'');
  },
  INVALID_ID: function INVALID_ID() {
    return new DiagramSchemaError('A valid node should have the required and unique property \'id\'');
  },
  INVALID_COORDS: function INVALID_COORDS(id) {
    return new DiagramSchemaError("".concat(id, " node 'coordinates' property is not valid."));
  },
  INVALID_CONTENT: function INVALID_CONTENT(id) {
    return new DiagramSchemaError("".concat(id, " node 'content' property is not valid."));
  },
  INVALID_PORT_ID: function INVALID_PORT_ID() {
    return new DiagramSchemaError('A valid port should have a unique id');
  },
  INVALID_PORT_CAN_LINK: function INVALID_PORT_CAN_LINK(id) {
    return new DiagramSchemaError("".concat(id, " port 'canLink' property is not valid."));
  },
  INVALID_PORT_ALIGNMENT: function INVALID_PORT_ALIGNMENT(id) {
    return new DiagramSchemaError("".concat(id, " port 'alignment' property is not valid."));
  },
  INVALID_INPUTS_ARRAY: function INVALID_INPUTS_ARRAY(id) {
    return new DiagramSchemaError("".concat(id, " node 'input' property is not valid."));
  },
  INVALID_NODES_ARRAY: function INVALID_NODES_ARRAY() {
    return new DiagramSchemaError('The \'nodes\' property is not a valid array');
  },
  INVALID_LINKS_ARRAY: function INVALID_LINKS_ARRAY() {
    return new DiagramSchemaError('The \'links\' property is not a valid array');
  },
  LINK_INVALID_INPUT_OUTPUT: function LINK_INVALID_INPUT_OUTPUT() {
    return new DiagramSchemaError('Link properties \'input\' and \'output\' are required string');
  },
  LINK_INVALID_READONLY: function LINK_INVALID_READONLY() {
    return new DiagramSchemaError('Link property \'readonly\' should be a boolean value');
  }
});

exports.DiagramSchemaError = DiagramSchemaError;
exports.default = ERR;
