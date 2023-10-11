/* beautiful-react-diagrams version: 0.5.1 */
'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

var React = require('react');
var Errors = require('../Errors.js');

var validatePort = function validatePort(port) {
  if (!port.id) {
    throw Errors['default'].INVALID_PORT_ID();
  }

  if (!!port.canLink && typeof port.canLink !== 'function') {
    throw Errors['default'].INVALID_PORT_CAN_LINK(port.id);
  }

  if (!!port.alignment && !['right', 'left', 'top', 'bottom'].includes(port.alignment)) {
    throw Errors['default'].INVALID_PORT_ALIGNMENT(port.id);
  }

  return true;
};
var validateNode = function validateNode(node) {
  if (!node.id) {
    throw Errors['default'].INVALID_ID();
  }

  if (!node.coordinates || !Array.isArray(node.coordinates) || node.coordinates.length !== 2) {
    throw Errors['default'].INVALID_COORDS(node.id);
  }

  if (!!node.content && typeof node.content !== 'string' && typeof node.content !== 'function' && !React.isValidElement(node.content)) {
    throw Errors['default'].INVALID_CONTENT(node.id);
  }

  if (node.inputs) {
    if (!Array.isArray(node.inputs)) {
      throw Errors['default'].INVALID_INPUTS_ARRAY(node.id);
    }

    node.inputs.forEach(validatePort);
  }

  if (node.outputs) {
    if (!Array.isArray(node.outputs)) {
      throw Errors['default'].INVALID_INPUTS_ARRAY(node.id);
    }

    node.outputs.forEach(validatePort);
  }

  return true;
};
var validateNodes = function validateNodes(nodes) {
  if (!Array.isArray(nodes)) {
    throw Errors['default'].INVALID_NODES_ARRAY();
  }

  nodes.forEach(validateNode);
  return true;
};
var validateLink = function validateLink(link) {
  if (!link.input || !link.output || typeof link.input !== 'string' || typeof link.output !== 'string') {
    throw Errors['default'].LINK_INVALID_INPUT_OUTPUT();
  }

  if (link.readonly && typeof link.readonly !== 'boolean') {
    throw Errors['default'].LINK_INVALID_READONLY();
  }

  return true;
};
var validateLinks = function validateLinks(links) {
  if (!Array.isArray(links)) {
    throw Errors['default'].INVALID_LINKS_ARRAY();
  }

  links.forEach(validateLink);
  return true;
};
var validateSchema = function validateSchema(_ref) {
  var links = _ref.links,
      nodes = _ref.nodes;

  if (!nodes) {
    throw Errors['default'].MUST_HAVE_NODES();
  }

  return validateLinks(links) && validateNodes(nodes);
};

exports.default = validateSchema;
exports.validateLink = validateLink;
exports.validateLinks = validateLinks;
exports.validateNode = validateNode;
exports.validateNodes = validateNodes;
exports.validatePort = validatePort;
exports.validateSchema = validateSchema;
