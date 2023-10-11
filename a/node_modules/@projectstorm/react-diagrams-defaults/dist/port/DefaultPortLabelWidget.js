import * as React from 'react';
import { PortWidget } from '@projectstorm/react-diagrams-core';
import styled from '@emotion/styled';
var S;
(function (S) {
    S.PortLabel = styled.div `
		display: flex;
		margin-top: 1px;
		align-items: center;
	`;
    S.Label = styled.div `
		padding: 0 5px;
		flex-grow: 1;
	`;
    S.Port = styled.div `
		width: 15px;
		height: 15px;
		background: rgba(255, 255, 255, 0.1);

		&:hover {
			background: rgb(192, 255, 0);
		}
	`;
})(S || (S = {}));
export class DefaultPortLabel extends React.Component {
    render() {
        const port = (React.createElement(PortWidget, { engine: this.props.engine, port: this.props.port },
            React.createElement(S.Port, null)));
        const label = React.createElement(S.Label, null, this.props.port.getOptions().label);
        return (React.createElement(S.PortLabel, null,
            this.props.port.getOptions().in ? port : label,
            this.props.port.getOptions().in ? label : port));
    }
}
//# sourceMappingURL=DefaultPortLabelWidget.js.map