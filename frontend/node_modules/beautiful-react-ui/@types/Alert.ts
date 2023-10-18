import { FunctionComponent } from 'react';
import { Callback, Color, DefaultProps } from './_shared';

export type AlertProps = DefaultProps & {
  /**
   * Defines the color of the alert, can be `default`, `primary`, `secondary`, `info`, `warning`, `success`, `danger`.
   */
  color?: Color,
  /**
   * A solid background style variant with white text and without border.
   */
  solid?: boolean,
  /**
   * Shows the outlines only
   */
  outline?: boolean,
  /**
   * onClose accept a function. If there's any function, it will show a button
   */
  onClose?: Callback<Event>,
};


declare const Alert: FunctionComponent<AlertProps>;

export default Alert;
