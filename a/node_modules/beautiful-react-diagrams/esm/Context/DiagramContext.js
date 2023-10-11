/* beautiful-react-diagrams version: 0.5.1 */
import React from 'react';

var DiagramContext = React.createContext({
  canvas: null,
  ports: null,
  nodes: null
});

export default DiagramContext;
