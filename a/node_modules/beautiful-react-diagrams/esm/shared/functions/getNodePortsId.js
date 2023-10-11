/* beautiful-react-diagrams version: 0.5.1 */
var getNodePortsId = function getNodePortsId(node, portType) {
  if (node[portType] && node[portType].length > 0) {
    return node[portType].map(function (port) {
      return port.id;
    });
  }

  return [];
};

export default getNodePortsId;
