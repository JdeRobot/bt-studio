import JSZip from "jszip";
import { getFileList, getWorldFile } from "BtApi/TreeWrapper";
import { Entry } from "jderobot-ide-interface";

const zipFile = async (
  zip: JSZip,
  project: string,
  world_name: string,
  file_path: string,
  file_name: string,
) => {
  const content = await getWorldFile(project, world_name, file_path);
  zip.file(file_name, content);
};

const zipFolder = async (
  zip: JSZip,
  file: Entry,
  project: string,
  world_name: string,
) => {
  const folder = zip.folder(file.name);

  if (folder === null) {
    return;
  }

  for (let index = 0; index < file.files.length; index++) {
    const element = file.files[index];
    console.log(element);
    if (element.is_dir) {
      await zipFolder(folder, element, project, world_name);
    } else {
      await zipFile(folder, project, world_name, element.path, element.name);
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

export const zipCustomWorld = async (project: string, name: string) => {
  const file_list = await getFileList(project, name);

  const files: Entry[] = JSON.parse(file_list);

  const base_entry: Entry = {
    name: name,
    is_dir: true,
    path: "",
    files: files,
    group: "",
    access: false,
  };

  const zip = new JSZip();

  for (let index = 0; index < base_entry.files.length; index++) {
    const element = base_entry.files[index];
    if (element.is_dir) {
      await zipFolder(zip, element, project, base_entry.name);
    } else {
      await zipFile(zip, project, base_entry.name, element.path, element.name);
    }
  }

  const base64data = await zipToData(zip);

  return base64data;
};
