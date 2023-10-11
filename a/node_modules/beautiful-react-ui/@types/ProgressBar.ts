import { FunctionComponent, ReactElement } from 'react';
import { Color, DefaultProps } from './_shared';

export type ProgressBarProps = DefaultProps & {
  /**
   * Defines the percentage of the completed progress
   */
  completed?: number,
  /**
   * Defines the line colour
   */
  color?: Color,
  /**
   * A renderer to replace the standard sidebar element
   */
  ElementRender?: ReactElement,
  /**
   * A renderer to replace the standard line element
   */
  FillRender?: ReactElement,
};

/**
 * Declares the Input functional component
 */
declare const ProgressBar: FunctionComponent<ProgressBarProps>;

export default ProgressBar;
