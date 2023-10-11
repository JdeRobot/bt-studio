import { AbstractDisplacementState, Action, InputType } from '@projectstorm/react-canvas-core';
import { PortModel } from '../entities/port/PortModel';
export class DragNewLinkState extends AbstractDisplacementState {
    constructor(options = {}) {
        super({ name: 'drag-new-link' });
        this.config = Object.assign({ allowLooseLinks: true, allowLinksFromLockedPorts: false }, options);
        this.registerAction(new Action({
            type: InputType.MOUSE_DOWN,
            fire: (event) => {
                this.port = this.engine.getMouseElement(event.event);
                if (!this.config.allowLinksFromLockedPorts && this.port.isLocked()) {
                    this.eject();
                    return;
                }
                this.link = this.port.createLinkModel();
                // if no link is given, just eject the state
                if (!this.link) {
                    this.eject();
                    return;
                }
                this.link.setSelected(true);
                this.link.setSourcePort(this.port);
                this.engine.getModel().addLink(this.link);
                this.port.reportPosition();
            }
        }));
        this.registerAction(new Action({
            type: InputType.MOUSE_UP,
            fire: (event) => {
                const model = this.engine.getMouseElement(event.event);
                // check to see if we connected to a new port
                if (model instanceof PortModel) {
                    if (this.port.canLinkToPort(model)) {
                        this.link.setTargetPort(model);
                        model.reportPosition();
                        this.engine.repaintCanvas();
                        return;
                    }
                    else {
                        this.link.remove();
                        this.engine.repaintCanvas();
                        return;
                    }
                }
                if (!this.config.allowLooseLinks) {
                    this.link.remove();
                    this.engine.repaintCanvas();
                }
            }
        }));
    }
    /**
     * Calculates the link's far-end point position on mouse move.
     * In order to be as precise as possible the mouse initialXRelative & initialYRelative are taken into account as well
     * as the possible engine offset
     */
    fireMouseMoved(event) {
        const portPos = this.port.getPosition();
        const zoomLevelPercentage = this.engine.getModel().getZoomLevel() / 100;
        const engineOffsetX = this.engine.getModel().getOffsetX() / zoomLevelPercentage;
        const engineOffsetY = this.engine.getModel().getOffsetY() / zoomLevelPercentage;
        const initialXRelative = this.initialXRelative / zoomLevelPercentage;
        const initialYRelative = this.initialYRelative / zoomLevelPercentage;
        const linkNextX = portPos.x - engineOffsetX + (initialXRelative - portPos.x) + event.virtualDisplacementX;
        const linkNextY = portPos.y - engineOffsetY + (initialYRelative - portPos.y) + event.virtualDisplacementY;
        this.link.getLastPoint().setPosition(linkNextX, linkNextY);
        this.engine.repaintCanvas();
    }
}
//# sourceMappingURL=DragNewLinkState.js.map