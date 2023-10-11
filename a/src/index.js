import React from 'react';
import ReactDOM from 'react-dom/client';
import WebFont from 'webfontloader';
import './index.css';
import App from './App';
import reportWebVitals from './reportWebVitals';
import 'beautiful-react-diagrams/styles.css';

// Load Google Font
WebFont.load({
  google: {
    families: ['DM+Sans:300,400,700', 'Spline+Sans+Mono:400', 'Spline+Sans:400', 'sans-serif'],
  },
});

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
