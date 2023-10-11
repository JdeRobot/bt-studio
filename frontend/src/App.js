// App.js
import React, { useState } from 'react';
import { Resizable } from 'react-resizable';
import HeaderMenu from './components/HeaderMenu';
import FileBrowser from './components/FileBrowser';
import FileEditor from './components/FileEditor';
import DiagramEditor from './components/DiagramEditor';
import './App.css';

const App = () => {
  const [browserWidth, setBrowserWidth] = useState(150);
  const [editorWidth, setEditorWidth] = useState(600);

  const [currentFileContent, setCurrentFileContent] = useState('');

  const onResize = (key, size) => {
    switch (key) {
      case 'editorWidth':
        setEditorWidth(size.width);
        break;
      default:
        break;
    }
  };

  return (
    <div className="App">
      <HeaderMenu />
      <div className="App-main" style={{ display: 'flex' }}>
        <div style={{ flex: `0 0 ${browserWidth}px` }}>
          <FileBrowser setCurrentFileContent={setCurrentFileContent} />
        </div>
        
        <Resizable
          width={editorWidth}
          height={0}
          onResize={(e, { size }) => onResize('editorWidth', size)}
          minConstraints={[400, 400]}
          maxConstraints={[800, 800]}
        >
          <div style={{ flex: `0 0 ${editorWidth}px` }}>
            <FileEditor currentFileContent={currentFileContent} />
          </div>
        </Resizable>

        <div style={{ flex: 1, paddingRight: "1vh"}}>
          <DiagramEditor />
        </div>
      </div>
    </div>
  );
};

export default App;
