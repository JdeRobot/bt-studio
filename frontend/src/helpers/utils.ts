import { getFile } from "BtApi/TreeWrapper";
import { Entry } from "jderobot-ide-interface";
import JSZip from "jszip";
import { MutableRefObject } from "react";

export function subscribe(eventName: string, listener: (e: unknown) => void) {
  document.addEventListener(eventName, listener);
}

export function unsubscribe(eventName: string, listener: () => void) {
  document.removeEventListener(eventName, listener);
}

export function publish(eventName: string, extra?: unknown) {
  const event = new CustomEvent(eventName, { detail: extra });
  document.dispatchEvent(event);
}

export const zipCodeFiles = async (
  zip: JSZip,
  files: Entry[],
  project: string,
  world?: string,
) => {
  for (const file of files) {
    if (file.is_dir) {
      await zipCodeFolder(zip, file, project, world);
    } else {
      await zipCodeFile(zip, file, project, world);
    }
  }
};

const zipCodeFile = async (
  zip: JSZip,
  file: Entry,
  project: string,
  world?: string,
) => {
  const content = await getFile(project, file.path, world, file.binary);
  zip.file(file.name, content, { binary: file.binary });
};

const zipCodeFolder = async (
  zip: JSZip,
  file: Entry,
  project: string,
  world?: string,
) => {
  const folder = zip.folder(file.name);

  if (folder === null) {
    return;
  }

  for (let index = 0; index < file.files.length; index++) {
    const element = file.files[index];
    if (element.is_dir) {
      await zipCodeFolder(folder, element, project, world);
    } else {
      await zipCodeFile(folder, element, project, world);
    }
  }
};

export const clearTimeouts = (
  timeoutsRef: MutableRefObject<number | null>[],
) => {
  for (const element of timeoutsRef) {
    if (element.current) {
      window.clearTimeout(element.current);
    }
  }
};
