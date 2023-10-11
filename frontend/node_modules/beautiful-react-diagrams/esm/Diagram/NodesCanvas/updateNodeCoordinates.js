/* beautiful-react-diagrams version: 0.5.1 */
import findIndex from 'lodash.findindex';

var updateNodeCoordinates = function updateNodeCoordinates(nodeId, coordinates, nodes) {
  var index = findIndex(nodes, ['id', nodeId]);

  if (index > -1 && !nodes[index].disableDrag) {
    nodes[index].coordinates = coordinates;
  }

  return nodes;
};

export default updateNodeCoordinates;
