import { FunctionComponent, ReactElement, ReactNode } from 'react';
import { Callback, DefaultProps, Placement } from './_shared';


export type FloatingContentProps = DefaultProps & {
  /**
   * Defines the React node to apply the floating content to
   */
  trigger: ReactNode | ReactElement,
  /**
   * Defines whether the floating content is shown or not
   */
  isShown?: boolean,
  /**
   * Defines the callback to be performed each time the event defined by the `action` prop fires,
   * by default a `click` event
   */
  onToggle: Callback,
  /**
   * Defines when to fire the onToggle callback, it can be `click` or `hover`
   */
  action?: 'click' | 'hover',
  /**
   * If the `action` prop is set to `click`, it's possible to define if the `onToggle` callback should be performed
   * when clicking outside of the trigger
   */
  clickOutsideToToggle?: boolean,
  /**
   * Defines the popup placement
   */
  placement?: Placement,
  /**
   * Defines a number in pixel to possibly separate the popup from the trigger
   */
  offset?: number,
  /**
   * Defines if the floating content should have the same width of its trigger
   */
  widthAsTrigger?: boolean,
  /**
   * Defines if the floating content placement must be reversed if there's not enough space to show it
   */
  reversePlacementOnSmallSpace?: boolean,
};

/**
 * Declares the Input functional component
 */
declare const FloatingContent: FunctionComponent<FloatingContentProps>;

export default FloatingContent;
