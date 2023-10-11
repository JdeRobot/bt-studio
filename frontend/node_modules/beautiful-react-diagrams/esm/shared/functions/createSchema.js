/* beautiful-react-diagrams version: 0.5.1 */
import { objectSpread2 as _objectSpread2 } from '../../_virtual/_rollupPluginBabelHelpers.js';
import ensureNodeId from './ensureNodeId.js';
import { validateSchema } from './validators.js';

var createSchema = function createSchema(schema) {
  var next = _objectSpread2({}, schema);

  next.nodes || (next.nodes = []);
  next.links || (next.links = []);
  next.nodes.forEach(ensureNodeId);
  validateSchema(next);
  return next;
};

export default createSchema;
