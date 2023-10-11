import * as React from 'react';
import styled from '@emotion/styled';
var S;
(function (S) {
    S.PointTop = styled.circle `
		pointer-events: all;
	`;
})(S || (S = {}));
export class DefaultLinkPointWidget extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            selected: false
        };
    }
    render() {
        const { point } = this.props;
        return (React.createElement("g", null,
            React.createElement("circle", { cx: point.getPosition().x, cy: point.getPosition().y, r: 5, fill: this.state.selected || this.props.point.isSelected() ? this.props.colorSelected : this.props.color }),
            React.createElement(S.PointTop, { className: "point", onMouseLeave: () => {
                    this.setState({ selected: false });
                }, onMouseEnter: () => {
                    this.setState({ selected: true });
                }, "data-id": point.getID(), "data-linkid": point.getLink().getID(), cx: point.getPosition().x, cy: point.getPosition().y, r: 15, opacity: 0.0 })));
    }
}
//# sourceMappingURL=DefaultLinkPointWidget.js.map