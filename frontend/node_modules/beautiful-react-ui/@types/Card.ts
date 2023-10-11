import { FunctionComponent, ReactElement } from 'react';
import { DefaultProps, IconProp } from './_shared';
import { TitleProps } from "./Title";
import { ImageProps } from "./Image";

export type CardTitleProps = TitleProps;

export type CardAlignableContentProps = { textAlign?: 'center' | 'left' | 'right' | 'justify' };

export type CardImageProps = ImageProps;

export type CardProps = DefaultProps & {
  /**
   * Allows to align card text content
   */
  textAlign?: 'center' | 'left' | 'right' | 'justify',
  /**
   * Defines if the card should adapt its width to its container or not
   */
  fluid?: boolean,
  /**
   * Defines the card orientation
   */
  orientation?: 'horizontal' | 'vertical',
  /**
   * Shows an overlapping opaque layer with a Spinner in the middle
   */
  loading?: boolean,
  /**
   * If true, it shows an actionButton that will run a callback
   */
  actionButton?: boolean,
  /**
   * Allows to change actionButton's icon
   */
  actionButtonIcon?: IconProp,
  /**
   * The callback to be performed on action button click
   */
  onActionButtonClick?: ReactElement,
  /**
   * Defines weather the card should reverse its column or not
   */
  reversed?: boolean,
  /**
   * Defines weather the card should float on mouse hover or not
   */
  float?: boolean,
  /**
   * Allows to change the standard action button behaviour by defining a custom renderer
   */
  actionButtonRenderer?: ReactElement,
  /**
   * Allows to change the standard card's image behaviour by defining a custom renderer
   */
  imageRenderer?: ReactElement,
};

/**
 * Declares the Card functional component
 */
declare const Card: FunctionComponent<CardProps>
  & { Title: FunctionComponent<CardTitleProps> }
  & { Content: FunctionComponent<CardAlignableContentProps> }
  & { Image: FunctionComponent<CardImageProps> }
  & { Footer: FunctionComponent<CardAlignableContentProps> };

export default Card;
