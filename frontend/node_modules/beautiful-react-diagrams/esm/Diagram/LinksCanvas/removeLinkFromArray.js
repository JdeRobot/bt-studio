/* beautiful-react-diagrams version: 0.5.1 */
import isEqual from 'lodash.isequal';

var removeLinkFromArray = function removeLinkFromArray(link, links) {
  return links.filter(function (item) {
    return !isEqual(item, link);
  });
};

export default removeLinkFromArray;
