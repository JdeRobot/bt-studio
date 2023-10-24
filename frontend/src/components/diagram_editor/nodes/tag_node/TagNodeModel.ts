import { DefaultNodeModel, DefaultPortModel } from '@projectstorm/react-diagrams';

export class TagNodeModel extends DefaultNodeModel {

    constructor(name: string = 'Tag Node', color: string = 'rgb(0,192,255)') {
        super({
            type: "tag",
            name: name,
            color: color,
        });
    }

    // Method to add children port (they can be default model because only the widget, the visualization changes)
    addOutputPort(name: string) {
        const port = new DefaultPortModel({
            in: false,
            name: name,
            label: name,
            type: 'output port'
        });
        this.addPort(port);
        return port;
    }
    
    // Method to add a special port
    addInputPort(name: string) {
        const port = new DefaultPortModel({
            in: true,
            name: name,
            label: name,
            type: 'input port'
        });
        this.addPort(port);
        return port;
    }
}
