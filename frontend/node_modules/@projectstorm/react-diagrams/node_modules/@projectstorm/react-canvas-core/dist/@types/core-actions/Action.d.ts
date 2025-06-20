import { MouseEvent, KeyboardEvent, WheelEvent, TouchEvent, SyntheticEvent } from 'react';
import { CanvasEngine } from '../CanvasEngine';
import { BaseModel } from '../core-models/BaseModel';
export declare enum InputType {
    MOUSE_DOWN = "mouse-down",
    MOUSE_UP = "mouse-up",
    MOUSE_MOVE = "mouse-move",
    MOUSE_WHEEL = "mouse-wheel",
    KEY_DOWN = "key-down",
    KEY_UP = "key-up",
    TOUCH_START = "touch-start",
    TOUCH_END = "touch-end",
    TOUCH_MOVE = "touch-move"
}
export interface Mapping {
    [InputType.MOUSE_DOWN]: MouseEvent;
    [InputType.MOUSE_UP]: MouseEvent;
    [InputType.MOUSE_MOVE]: MouseEvent;
    [InputType.MOUSE_WHEEL]: WheelEvent;
    [InputType.KEY_DOWN]: KeyboardEvent;
    [InputType.KEY_UP]: KeyboardEvent;
    [InputType.TOUCH_START]: TouchEvent;
    [InputType.TOUCH_END]: TouchEvent;
    [InputType.TOUCH_MOVE]: TouchEvent;
}
export interface ActionEvent<Event extends SyntheticEvent = SyntheticEvent, Model extends BaseModel = BaseModel> {
    event: Event;
    model?: Model;
}
export interface ActionOptions {
    type: InputType;
    fire: (event: ActionEvent<Mapping[this['type']]>) => void;
}
export declare class Action<T extends CanvasEngine = CanvasEngine> {
    options: ActionOptions;
    id: string;
    engine: T;
    constructor(options: ActionOptions);
    setEngine(engine: T): void;
}
