/* beautiful-react-diagrams version: 0.5.1 */
import React from 'react';
import PropTypes from 'prop-types';

var LinkLabel = function LinkLabel(_ref) {
  var label = _ref.label,
      position = _ref.position;
  return React.createElement("foreignObject", {
    x: position[0],
    y: position[1]
  }, React.createElement("div", {
    className: "bi-diagram-link-label"
  }, label));
};

LinkLabel.propTypes = {
  label: PropTypes.string.isRequired,
  position: PropTypes.arrayOf(PropTypes.number).isRequired
};
var LinkLabel$1 = React.memo(LinkLabel);

export default LinkLabel$1;
