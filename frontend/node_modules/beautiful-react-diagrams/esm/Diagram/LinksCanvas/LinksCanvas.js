/* beautiful-react-diagrams version: 0.5.1 */
import React, { useCallback } from 'react';
import PropTypes from 'prop-types';
import { NodeType, LinkType, PortAlignment } from '../../shared/Types.js';
import DiagramLink from '../Link/Link.js';
import Segment from '../Segment/Segment.js';
import findInvolvedEntity from './findInvolvedEntity.js';
import removeLinkFromArray from './removeLinkFromArray.js';

var LinksCanvas = function LinksCanvas(props) {
  var nodes = props.nodes,
      segment = props.segment,
      onChange = props.onChange,
      links = props.links;
  var removeFromLinksArray = useCallback(function (link) {
    if (links.length > 0 && onChange) {
      var nextLinks = removeLinkFromArray(link, links);
      onChange(nextLinks);
    }
  }, [links, onChange]);
  return React.createElement("svg", {
    className: "bi bi-link-canvas-layer"
  }, links && links.length > 0 && links.map(function (link) {
    return React.createElement(DiagramLink, {
      link: link,
      input: findInvolvedEntity(nodes, link.input),
      output: findInvolvedEntity(nodes, link.output),
      onDelete: removeFromLinksArray,
      key: "".concat(link.input, "-").concat(link.output)
    });
  }), segment && React.createElement(Segment, segment));
};

LinksCanvas.propTypes = {
  nodes: PropTypes.arrayOf(NodeType),
  links: PropTypes.arrayOf(LinkType),
  segment: PropTypes.exact({
    id: PropTypes.string,
    from: PropTypes.arrayOf(PropTypes.number),
    to: PropTypes.arrayOf(PropTypes.number),
    alignment: PortAlignment
  }),
  onChange: PropTypes.func
};
LinksCanvas.defaultProps = {
  nodes: [],
  links: [],
  segment: undefined,
  onChange: undefined
};
var LinksCanvas$1 = React.memo(LinksCanvas);

export default LinksCanvas$1;
