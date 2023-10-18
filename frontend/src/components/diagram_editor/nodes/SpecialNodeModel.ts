import { DefaultNodeModel, DefaultPortModel } from '@projectstorm/react-diagrams';

export class SpecialNodeModel extends DefaultNodeModel {

    constructor(name: string = 'Special Node', color: string = 'rgb(0,192,255)') {
        super({
            type: 'special',
            name: name,
            color: color,
        });
    }

    // Method to add children port (they can be default model because only the widget, the visualization changes)
    addChildrenPort(name: string) {
        const port = new DefaultPortModel({
            in: false,
            name: name,
            label: name,
            type: 'children port'
        });
        this.addPort(port);
        return port;
    }
    
    // Method to add a special port
    addParentPort(name: string) {
        const port = new DefaultPortModel({
            in: true,
            name: name,
            label: name,
            type: 'parent port'
        });
        this.addPort(port);
        return port;
    }

    // Method to add a normal port
    addNormalPort(name: string) {
        const port = new DefaultPortModel({
            in: false,
            name: name,
            label: name,
            type: 'default'
        });
        this.addPort(port);
        return port;
    }
}
