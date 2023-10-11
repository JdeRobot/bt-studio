import * as React from 'react';
import { SimpleClientRect } from '../../states/SelectionBoxState';
export interface SelectionBoxWidgetProps {
    rect: SimpleClientRect;
}
export declare class SelectionBoxWidget extends React.Component<SelectionBoxWidgetProps> {
    render(): React.JSX.Element;
}
