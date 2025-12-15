import {
  getFileList,
  createFile,
  getFile,
  renameFile,
  deleteFile,
  uploadFile,
  createFolder,
  renameFolder,
  deleteFolder,
  createAction,
} from "BtApi/TreeWrapper";
import { CreateAction, newFileData } from "../CreateAction";
import { publish } from "../helper/TreeEditorHelper";

const fileExplorer = {
  name: "Code",
  list: (project: string) => {
    return getFileList(project);
  },
  file: {
    create: (project: string, location: string, name: string) => {
      return createFile(project, name, location);
    },
    get: (project: string, path: string) => {
      return getFile(project, path);
    },
    rename: (project: string, oldPath: string, newPath: string) => {
      return renameFile(project, oldPath, newPath);
    },
    delete: (project: string, path: string) => {
      return deleteFile(project, path);
    },
    upload: (project: string, path: string, name: string, content: string) => {
      return uploadFile(project, name, path, content);
    },
  },
  folder: {
    create: (project: string, location: string, name: string) => {
      return createFolder(project, name, location);
    },
    rename: (project: string, oldPath: string, newPath: string) => {
      return renameFolder(project, oldPath, newPath);
    },
    delete: (project: string, path: string) => {
      return deleteFolder(project, path);
    },
  },
  modals: {
    createFile: {
      component: CreateAction,
      onCreate: (project: string, location: string, data: newFileData) => {
        if (data.fileType === "actions") {
          publish("updateActionList");
          return createAction(project, data.fileName, data.templateType);
        } else {
          return createFile(project, data.fileName, location);
        }
      },
    },
  },
};

const universeExplorer = {
  name: "Universes",
  list: (project: string) => {
    return getFileList(project, "");
  },
  file: {
    create: (project: string, location: string, name: string) => {
      return createFile(project, name, location, "");
    },
    get: (project: string, path: string) => {
      return getFile(project, path, "");
    },
    rename: (project: string, oldPath: string, newPath: string) => {
      return renameFile(project, oldPath, newPath, "");
    },
    delete: (project: string, path: string) => {
      console.log(project, path);
      return deleteFile(project, path, "");
    },
    upload: (project: string, path: string, name: string, content: string) => {
      return uploadFile(project, name, path, content, "");
    },
  },
  folder: {
    create: (project: string, location: string, name: string) => {
      return createFolder(project, name, location, "");
    },
    rename: (project: string, oldPath: string, newPath: string) => {
      return renameFolder(project, oldPath, newPath, "");
    },
    delete: (project: string, path: string) => {
      return deleteFolder(project, path, "");
    },
  },
};

const explorers=[fileExplorer, universeExplorer]

export default explorers;