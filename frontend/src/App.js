// App.js
import React, { useState } from 'react';
import { Resizable } from 'react-resizable';
import HeaderMenu from './components/header_menu/HeaderMenu';
import FileBrowser from './components/file_browser/FileBrowser';
import FileEditor from './components/file_editor/FileEditor';
import './App.css';
import DiagramEditor from './components/diagram_editor/DiagramEditor';

const App = () => {

  const [editorWidth, setEditorWidth] = useState(700);
  const [currentFilename, setCurrentFilename] = useState('');
  const [currentProjectname, setCurrentProjectname] = useState('');
  const [modelJson, setModelJson] = useState('');
  const [projectChanges, setProjectChanges] = useState(false);

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

      <HeaderMenu 
        setCurrentProjectname={setCurrentProjectname} 
        currentProjectname={currentProjectname}
        modelJson={modelJson}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
      />

      <div className="App-main" style={{ display: 'flex' }}>

        <div style={{ width: '200px'}}>
          <FileBrowser 
            setCurrentFilename={setCurrentFilename} 
            currentFilename={currentFilename}
            currentProjectname={currentProjectname}
            setProjectChanges={setProjectChanges}
          />
        </div>
        
        <Resizable
          width={editorWidth}
          height={0}
          onResize={(e, { size }) => onResize('editorWidth', size)}
          minConstraints={[400, 400]}
          maxConstraints={[900, 900]}
        >
          <div style={{ width: `${editorWidth}px` }}>
            <FileEditor 
              currentFilename = {currentFilename} 
              currentProjectname={currentProjectname}
              setProjectChanges={setProjectChanges}
            />
          </div>
        </Resizable>

        <div style={{ flex: 1}}>
          <DiagramEditor 
            currentProjectname={currentProjectname}
            setModelJson={setModelJson}
            setProjectChanges={setProjectChanges}
          />
        </div>
        
      </div>

    </div>
  );
};

export default App;
