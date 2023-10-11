import { FunctionComponent } from 'react';
import { DefaultProps } from './_shared';

export type ImageProps = DefaultProps & {
  /**
   * The image source
   */
  src: string,
  /**
   * The image alternative text
   */
  alt: string,
  /**
   * Applies the thumbnail style to the image
   */
  thumb?: boolean,
  /**
   * Applies a fully rounded style to the image
   */
  rounded?: boolean,
};

declare const Image: FunctionComponent<ImageProps>;

export default Image;
