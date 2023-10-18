import React from 'react';
import AceEditor from 'react-ace';
import 'ace-builds/src-noconflict/mode-python';
import 'ace-builds/src-noconflict/theme-monokai';

const FileEditor = ({ currentFileContent }) => {
  return (
    <div style={{flex: '2', border: '1px solid black'}}>
      <h2>File Editor</h2>
      <AceEditor
        mode="python"
        theme="monokai"
        name="fileEditor"
        width="100%"
        height="80vh"
        value={currentFileContent}
      />
    </div>
  );
};

export default FileEditor;
