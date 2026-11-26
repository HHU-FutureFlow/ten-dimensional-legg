import React from 'react';
import { createRoot } from 'react-dom/client';
import '@xyflow/react/dist/style.css';
import './style.css';
import './appearance.css';
import App from './App.tsx';

createRoot(document.getElementById('root')!).render(<React.StrictMode><App/></React.StrictMode>);
