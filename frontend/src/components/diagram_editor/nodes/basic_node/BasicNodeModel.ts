import { DefaultNodeModel, DefaultPortModel, NodeModel, NodeModelGenerics, PortModelAlignment } from '@projectstorm/react-diagrams';
import { ParentPortModel } from './ports/parent_port/ParentPortModel';
import { ChildrenPortModel } from './ports/children_port/ChildrenPortModel';

export interface BasicNodeModelGenerics {
    PORT: ParentPortModel | ChildrenPortModel;
}

export class BasicNodeModel extends NodeModel<NodeModelGenerics & BasicNodeModelGenerics> {

    private name: string;
    private color: string;

    constructor(name: string = 'Basic Node', color: string = 'rgb(0,192,255)') {
        super({
            type: "basic",
        });
        this.name = name;
        this.color = color;
    }

    getName(): string {
        return this.name;
    }

    getColor(): string {
        return this.color;
    }

    addChildrenPort(name: string) {
        const port = new ChildrenPortModel();
        this.addPort(port);
        return port;
    }
    
    // Method to add a special port
    addParentPort(name: string) {
        const port = new ParentPortModel();
        this.addPort(port);
        return port;
    }

    // Method to add a normal port
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

    // Method to add a normal port
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
}
