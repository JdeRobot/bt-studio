import * as React from 'react';
import styled from '@emotion/styled';
var S;
(function (S) {
    S.Label = styled.div `
		background: rgba(0, 0, 0, 0.8);
		border-radius: 5px;
		color: white;
		font-size: 12px;
		padding: 4px 8px;
		font-family: sans-serif;
		user-select: none;
	`;
})(S || (S = {}));
export class DefaultLabelWidget extends React.Component {
    render() {
        return React.createElement(S.Label, null, this.props.model.getOptions().label);
    }
}
//# sourceMappingURL=DefaultLabelWidget.js.map