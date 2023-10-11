/* beautiful-react-diagrams version: 0.5.1 */
import { useContext } from 'react';
import DiagramContext from '../../Context/DiagramContext.js';

var useCanvas = function useCanvas() {
  var _useContext = useContext(DiagramContext),
      canvas = _useContext.canvas;

  return canvas;
};

export default useCanvas;
