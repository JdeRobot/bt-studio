import * as React from 'react';
import styled from '@emotion/styled';
import { css } from '@emotion/react';
var S;
(function (S) {
    const shared = css `
		top: 0;
		left: 0;
		right: 0;
		bottom: 0;
		position: absolute;
		pointer-events: none;
		transform-origin: 0 0;
		width: 100%;
		height: 100%;
		overflow: visible;
	`;
    S.DivLayer = styled.div `
		${shared}
	`;
    S.SvgLayer = styled.svg `
		${shared}
	`;
})(S || (S = {}));
export class TransformLayerWidget extends React.Component {
    constructor(props) {
        super(props);
        this.state = {};
    }
    getTransform() {
        const model = this.props.layer.getParent();
        return `
			translate(
				${model.getOffsetX()}px,
				${model.getOffsetY()}px)
			scale(
				${model.getZoomLevel() / 100.0}
			)
  	`;
    }
    getTransformStyle() {
        if (this.props.layer.getOptions().transformed) {
            return {
                transform: this.getTransform()
            };
        }
        return {};
    }
    render() {
        if (this.props.layer.getOptions().isSvg) {
            return React.createElement(S.SvgLayer, { style: this.getTransformStyle() }, this.props.children);
        }
        return React.createElement(S.DivLayer, { style: this.getTransformStyle() }, this.props.children);
    }
}
//# sourceMappingURL=TransformLayerWidget.js.map