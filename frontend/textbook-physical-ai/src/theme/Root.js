import React from 'react';
import { AuthProvider } from '../context/AuthContext';
import { ThemeProvider } from '../contexts/ThemeContext';
import BrowserOnly from '@docusaurus/BrowserOnly'; // Import BrowserOnly

export default function Root({ children }) {
  return (
    <BrowserOnly fallback={<div>Loading authentication...</div>}>
      {() => (
        <AuthProvider>
          <ThemeProvider>
            {children}
          </ThemeProvider>
        </AuthProvider>
      )}
    </BrowserOnly>
  );
}
