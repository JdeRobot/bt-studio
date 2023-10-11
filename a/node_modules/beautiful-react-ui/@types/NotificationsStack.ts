import { ElementType, FunctionComponent, ReactElement } from 'react';
import { Callback, DefaultProps } from "./_shared";
import { IconProps } from "./Icon";

export type NotificationObject = {
  id: number,
  content: string,
  title?: string,
  icon?: IconProps,
  avatar?: string,
  timeout?: boolean | number,
  closable?: boolean,
}

export type NotificationsStackProps = DefaultProps & {
  notifications: NotificationObject,
  onChange: Callback<MouseEvent>,
  position?: 'top-center' | 'bottom-center' | 'top-right' | 'top-left' | 'bottom-right' | 'bottom-left',
  NotificationRender?: ReactElement | ElementType,
  animation?: 'none' | 'fade' | 'zoom' | 'slide-right' | 'slide-top' | 'slide-left' | 'slide-bottom',
  color?: 'info' | 'success' | 'warning' | 'danger' | 'default',
};

/**
 * Declares the NotificationsStack functional component
 */
declare const NotificationsStack: FunctionComponent<NotificationsStackProps>;

export default NotificationsStack;