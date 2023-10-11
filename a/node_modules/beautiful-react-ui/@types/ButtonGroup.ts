import { FunctionComponent } from 'react';
import { Color, Size, DefaultProps } from './_shared';

export type ButtonGroupProps = DefaultProps & {
  /**
   * Defines the buttons color, can be `default`, `primary`, `secondary`, `info`, `warning`, `success`, `danger`
   * or `transparent`
   */
  color?: Color,
  /**
   * Defines the buttons' size, can be `small`, `default`, `large`
   */
  size?: Size,
  /**
   * Applies the outline style to the buttons
   */
  outline?: boolean,
  /**
   * Makes the buttons rounded
   */
  rounded?: boolean,
  /**
   * Makes the button completely fluid (full width)
   */
  fluid?: boolean,
};

declare const ButtonGroup: FunctionComponent<ButtonGroupProps>;

export default ButtonGroup;
