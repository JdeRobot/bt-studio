/* beautiful-react-diagrams version: 0.5.1 */
var getDiagramNodeStyle = function getDiagramNodeStyle(coordinates, disableDrag) {
  return {
    left: coordinates[0],
    top: coordinates[1],
    cursor: disableDrag ? undefined : 'move'
  };
};

export default getDiagramNodeStyle;
