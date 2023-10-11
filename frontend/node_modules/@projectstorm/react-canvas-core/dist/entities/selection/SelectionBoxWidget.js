import * as React from 'react';
import styled from '@emotion/styled';
var S;
(function (S) {
    S.Container = styled.div `
		position: absolute;
		background-color: rgba(0, 192, 255, 0.2);
		border: solid 2px rgb(0, 192, 255);
	`;
})(S || (S = {}));
export class SelectionBoxWidget extends React.Component {
    render() {
        const { rect } = this.props;
        if (!rect)
            return null;
        return (React.createElement(S.Container, { style: {
                top: rect.top,
                left: rect.left,
                width: rect.width,
                height: rect.height
            } }));
    }
}
//# sourceMappingURL=SelectionBoxWidget.js.map