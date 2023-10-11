import * as React from 'react';
import styled from '@emotion/styled';
var S;
(function (S) {
    S.Label = styled.div `
		display: inline-block;
		position: absolute;
	`;
    S.Foreign = styled.foreignObject `
		pointer-events: none;
		overflow: visible;
	`;
})(S || (S = {}));
export class LabelWidget extends React.Component {
    constructor(props) {
        super(props);
        this.findPathAndRelativePositionToRenderLabel = (index) => {
            // an array to hold all path lengths, making sure we hit the DOM only once to fetch this information
            const link = this.props.label.getParent();
            const lengths = link.getRenderedPath().map((path) => path.getTotalLength());
            // calculate the point where we want to display the label
            let labelPosition = lengths.reduce((previousValue, currentValue) => previousValue + currentValue, 0) *
                (index / (link.getLabels().length + 1));
            // find the path where the label will be rendered and calculate the relative position
            let pathIndex = 0;
            while (pathIndex < link.getRenderedPath().length) {
                if (labelPosition - lengths[pathIndex] < 0) {
                    return {
                        path: link.getRenderedPath()[pathIndex],
                        position: labelPosition
                    };
                }
                // keep searching
                labelPosition -= lengths[pathIndex];
                pathIndex++;
            }
        };
        this.calculateLabelPosition = () => {
            const found = this.findPathAndRelativePositionToRenderLabel(this.props.index + 1);
            if (!found) {
                return;
            }
            const { path, position } = found;
            const labelDimensions = {
                width: this.ref.current.offsetWidth,
                height: this.ref.current.offsetHeight
            };
            const pathCentre = path.getPointAtLength(position);
            const labelCoordinates = {
                x: pathCentre.x - labelDimensions.width / 2 + this.props.label.getOptions().offsetX,
                y: pathCentre.y - labelDimensions.height / 2 + this.props.label.getOptions().offsetY
            };
            this.ref.current.style.transform = `translate(${labelCoordinates.x}px, ${labelCoordinates.y}px)`;
        };
        this.ref = React.createRef();
    }
    componentDidUpdate() {
        window.requestAnimationFrame(this.calculateLabelPosition);
    }
    componentDidMount() {
        window.requestAnimationFrame(this.calculateLabelPosition);
    }
    render() {
        const canvas = this.props.engine.getCanvas();
        return (React.createElement(S.Foreign, { key: this.props.label.getID(), width: canvas === null || canvas === void 0 ? void 0 : canvas.offsetWidth, height: canvas === null || canvas === void 0 ? void 0 : canvas.offsetHeight },
            React.createElement(S.Label, { ref: this.ref }, this.props.engine.getFactoryForLabel(this.props.label).generateReactWidget({ model: this.props.label }))));
    }
}
//# sourceMappingURL=LabelWidget.js.map