import { LabelModel, LinkModel, PortModelAlignment } from '@projectstorm/react-diagrams-core';
import { DefaultLabelModel } from '../label/DefaultLabelModel';
import { BezierCurve } from '@projectstorm/geometry';
export class DefaultLinkModel extends LinkModel {
    constructor(options = {}) {
        super(Object.assign({ type: 'default', width: options.width || 3, color: options.color || 'gray', selectedColor: options.selectedColor || 'rgb(0,192,255)', curvyness: 50 }, options));
    }
    calculateControlOffset(port) {
        if (port.getOptions().alignment === PortModelAlignment.RIGHT) {
            return [this.options.curvyness, 0];
        }
        else if (port.getOptions().alignment === PortModelAlignment.LEFT) {
            return [-this.options.curvyness, 0];
        }
        else if (port.getOptions().alignment === PortModelAlignment.TOP) {
            return [0, -this.options.curvyness];
        }
        return [0, this.options.curvyness];
    }
    getSVGPath() {
        if (this.points.length == 2) {
            const curve = new BezierCurve();
            curve.setSource(this.getFirstPoint().getPosition());
            curve.setTarget(this.getLastPoint().getPosition());
            curve.setSourceControl(this.getFirstPoint().getPosition().clone());
            curve.setTargetControl(this.getLastPoint().getPosition().clone());
            if (this.sourcePort) {
                curve.getSourceControl().translate(...this.calculateControlOffset(this.getSourcePort()));
            }
            if (this.targetPort) {
                curve.getTargetControl().translate(...this.calculateControlOffset(this.getTargetPort()));
            }
            return curve.getSVGCurve();
        }
    }
    serialize() {
        return Object.assign(Object.assign({}, super.serialize()), { width: this.options.width, color: this.options.color, curvyness: this.options.curvyness, selectedColor: this.options.selectedColor });
    }
    deserialize(event) {
        super.deserialize(event);
        this.options.color = event.data.color;
        this.options.width = event.data.width;
        this.options.curvyness = event.data.curvyness;
        this.options.selectedColor = event.data.selectedColor;
    }
    addLabel(label) {
        if (label instanceof LabelModel) {
            return super.addLabel(label);
        }
        let labelOb = new DefaultLabelModel();
        labelOb.setLabel(label);
        return super.addLabel(labelOb);
    }
    setWidth(width) {
        this.options.width = width;
        this.fireEvent({ width }, 'widthChanged');
    }
    setColor(color) {
        this.options.color = color;
        this.fireEvent({ color }, 'colorChanged');
    }
}
//# sourceMappingURL=DefaultLinkModel.js.map