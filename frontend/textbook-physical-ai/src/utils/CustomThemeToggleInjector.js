import React from 'react';
import ReactDOM from 'react-dom/client';
import ThemeToggleCustom from '../theme/ThemeToggleCustom';

// Function to inject the custom theme toggle into the navbar
export const injectCustomThemeToggle = () => {
  // Wait for the DOM to be ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeCustomThemeToggle);
  } else {
    initializeCustomThemeToggle();
  }
};

const initializeCustomThemeToggle = () => {
  const mountPoint = document.getElementById('custom-theme-toggle-mount');
  if (mountPoint) {
    // Check if we already have a root attached to avoid duplicates
    if (!mountPoint._reactRoot) {
      const root = ReactDOM.createRoot(mountPoint);
      root.render(<ThemeToggleCustom />);

      // Store reference to prevent duplicate mounting
      mountPoint._reactRoot = root;
    }
  }
};

// Call the initialization function
injectCustomThemeToggle();

// Export for potential use elsewhere
export default injectCustomThemeToggle;