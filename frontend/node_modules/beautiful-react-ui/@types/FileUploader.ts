import { FunctionComponent, ReactElement } from 'react';
import { Callback, DefaultProps, IconProp } from './_shared';

export type Upload = {
  file: File,
  byteSent?: number,
  uploading?: boolean,
}

export type FileUploaderProps = DefaultProps & {
  /**
   * The array of the current uploading files.
   */
  uploads?: Upload[],
  /**
   * The callback to be performed when input's value changes
   */
  onChange?: Callback<Event, Upload[]>,
  /**
   * The function to be performed whilst performing an upload.
   * Receives the file to upload and the 'next' callback.
   * The next callback should be performed to update the file state by passing the uploading state
   */
  uploadingFn?: Callback<File, Function>,
  /**
   * Defines the FileUploader content title
   */
  title?: string,
  /**
   * Defines the FileUploader content subtitle
   */
  subtitle?: string,
  /**
   * The icon to be displayed in the middle of the FileUploader content.
   * A valid Icon component name prop or the instance of an Icon component are both valid values.
   */
  icon?: IconProp,
  /**
   * Defines whether the component should allows the user to select more than one file
   */
  multiple?: boolean,
  /**
   * Defines one or more unique file type specifiers describing file types to allow
   */
  accept?: string,
  /**
   * The "remove" label locale
   */
  removeLabel?: string,
  /**
   * The "uploading" label locale
   */
  uploadingLabel?: string,
  /**
   * A renderer to replace the standard FileUploader element
   */
  ElementRender?: ReactElement,
  /**
   * A renderer to replace the standard Title component
   */
  TitleRender?: ReactElement,
  /**
   * A renderer to replace the standard Subtitle component
   */
  SubtitleRender?: ReactElement,
  /**
   * A renderer to replace the standard FileItem component
   */
  FileItemRender?: ReactElement,
  /**
   * A renderer to replace the standard List component
   */
  ListRender?: ReactElement,
};

/**
 * Declares the FileUploader functional component
 */
declare const FileUploader: FunctionComponent<FileUploaderProps>;

export default FileUploader;
