const fs = require('fs');
require('dotenv').config();

// Define environment variables
const envVars = {
  REACT_APP_BACKEND_URL: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001',
  REACT_APP_BACKEND_URL_LIVE: process.env.REACT_APP_BACKEND_URL_LIVE || 'https://hackaton-ai-textbook.vercel.app/',
  REACT_APP_AUTH_API_URL: process.env.REACT_APP_AUTH_API_URL || 'http://localhost:8001/api/auth'
};

// Create the env.js file in the static directory
const envJsContent = `window._env_ = ${JSON.stringify(envVars, null, 2)};`;

// Ensure the directory exists
const dir = './static/js';
if (!fs.existsSync(dir)){
    fs.mkdirSync(dir, { recursive: true });
}

fs.writeFileSync('./static/js/env.js', envJsContent);
console.log('Environment variables file created successfully!');