import * as React from 'react';
export class SmartLayerWidget extends React.Component {
    shouldComponentUpdate() {
        return this.props.layer.isRepaintEnabled();
    }
    render() {
        return this.props.engine.getFactoryForLayer(this.props.layer).generateReactWidget({ model: this.props.layer });
    }
}
//# sourceMappingURL=SmartLayerWidget.js.map