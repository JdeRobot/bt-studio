import { getFile, getFileList } from "BtApi/TreeWrapper";
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
      await zipFolder(zip, file, project, world);
    } else {
      await zipFile(zip, file, project, world);
    }
  }
};

const zipFile = async (
  zip: JSZip,
  file: Entry,
  project: string,
  world?: string,
) => {
  const content = await getFile(project, file.path, world, file.binary);
  zip.file(file.name, content, { binary: file.binary });
};

const zipFolder = async (
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
      await zipFolder(folder, element, project, world);
    } else {
      await zipFile(folder, element, project, world);
    }
  }
};

const zipToData = (zip: JSZip) => {
  return new Promise((resolve) => {
    const reader = new FileReader();
    reader.onloadend = () => resolve(reader.result);
    zip.generateAsync({ type: "blob" }).then(function (content) {
      reader.readAsDataURL(content);
    });
  });
};

export const zipCustomWorld = async (project: string, world: string) => {
  const file_list = await getFileList(project, world);
  const files: Entry[] = JSON.parse(file_list);

  const zip = new JSZip();

  for (const file of files) {
    if (file.is_dir) {
      await zipFolder(zip, file, project, world);
    } else {
      await zipFile(zip, file, project, world);
    }
  }

  const base64data = await zipToData(zip);
  return base64data;
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
