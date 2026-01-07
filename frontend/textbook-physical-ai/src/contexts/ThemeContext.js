import React, { createContext, useContext, useState, useEffect } from 'react';

// Create Theme Context
const ThemeContext = createContext();

// Theme Provider Component
export const ThemeProvider = ({ children }) => {
  const [interactiveTheme, setInteractiveTheme] = useState({
    mode: 'system', // 'light', 'dark', 'system'
    accentColor: '#3578e5', // Default blue accent
    fontSize: 'medium', // 'small', 'medium', 'large'
    contrast: 'normal', // 'normal', 'high'
    animations: true,
    interactiveElements: true,
  });

  // Load theme preferences from localStorage on mount
  useEffect(() => {
    const savedTheme = localStorage.getItem('interactiveTheme');
    if (savedTheme) {
      try {
        const parsedTheme = JSON.parse(savedTheme);
        setInteractiveTheme(parsedTheme);
      } catch (error) {
        console.error('Error loading theme preferences:', error);
      }
    }
  }, []);

  // Save theme preferences to localStorage when they change
  useEffect(() => {
    localStorage.setItem('interactiveTheme', JSON.stringify(interactiveTheme));
  }, [interactiveTheme]);

  // Function to update theme settings
  const updateTheme = (newSettings) => {
    setInteractiveTheme(prev => ({
      ...prev,
      ...newSettings
    }));
  };

  // Function to update accent color
  const updateAccentColor = (color) => {
    setInteractiveTheme(prev => ({
      ...prev,
      accentColor: color
    }));
  };

  // Function to update font size
  const updateFontSize = (size) => {
    setInteractiveTheme(prev => ({
      ...prev,
      fontSize: size
    }));
  };

  // Function to update contrast level
  const updateContrast = (contrast) => {
    setInteractiveTheme(prev => ({
      ...prev,
      contrast: contrast
    }));
  };

  // Function to toggle animations
  const toggleAnimations = () => {
    setInteractiveTheme(prev => ({
      ...prev,
      animations: !prev.animations
    }));
  };

  // Function to toggle interactive elements
  const toggleInteractiveElements = () => {
    setInteractiveTheme(prev => ({
      ...prev,
      interactiveElements: !prev.interactiveElements
    }));
  };

  // Apply theme styles to document root
  useEffect(() => {
    const root = document.documentElement;

    // Set accent color
    root.style.setProperty('--ifm-color-primary', interactiveTheme.accentColor);
    root.style.setProperty('--ifm-color-primary-dark', shadeColor(interactiveTheme.accentColor, -20));
    root.style.setProperty('--ifm-color-primary-darker', shadeColor(interactiveTheme.accentColor, -30));
    root.style.setProperty('--ifm-color-primary-darkest', shadeColor(interactiveTheme.accentColor, -40));
    root.style.setProperty('--ifm-color-primary-light', shadeColor(interactiveTheme.accentColor, 20));
    root.style.setProperty('--ifm-color-primary-lighter', shadeColor(interactiveTheme.accentColor, 30));
    root.style.setProperty('--ifm-color-primary-lightest', shadeColor(interactiveTheme.accentColor, 40));

    // Set font size
    root.style.fontSize = interactiveTheme.fontSize === 'small' ? '14px' :
                         interactiveTheme.fontSize === 'large' ? '18px' : '16px';

    // Set contrast
    root.style.setProperty('--ifm-contrast', interactiveTheme.contrast === 'high' ? '1.5' : '1');

    // Set animation preference
    root.style.setProperty('--ifm-animation-duration', interactiveTheme.animations ? '0.3s' : '0s');

    // Add contrast class if high contrast
    if (interactiveTheme.contrast === 'high') {
      root.classList.add('high-contrast');
    } else {
      root.classList.remove('high-contrast');
    }

    // Add animation class based on preference
    if (!interactiveTheme.animations) {
      root.classList.add('no-animations');
    } else {
      root.classList.remove('no-animations');
    }
  }, [interactiveTheme]);

  // Helper function to shade colors
  const shadeColor = (color, percent) => {
    let R = parseInt(color.substring(1, 3), 16);
    let G = parseInt(color.substring(3, 5), 16);
    let B = parseInt(color.substring(5, 7), 16);

    R = Math.min(255, Math.max(0, R + R * percent / 100));
    G = Math.min(255, Math.max(0, G + G * percent / 100));
    B = Math.min(255, Math.max(0, B + B * percent / 100));

    const RR = Math.round(R).toString(16).padStart(2, '0');
    const GG = Math.round(G).toString(16).padStart(2, '0');
    const BB = Math.round(B).toString(16).padStart(2, '0');

    return `#${RR}${GG}${BB}`;
  };

  const value = {
    ...interactiveTheme,
    updateTheme,
    updateAccentColor,
    updateFontSize,
    updateContrast,
    toggleAnimations,
    toggleInteractiveElements,
  };

  return (
    <ThemeContext.Provider value={value}>
      {children}
    </ThemeContext.Provider>
  );
};

// Custom hook to use theme context
export const useTheme = () => {
  const context = useContext(ThemeContext);
  if (!context) {
    throw new Error('useTheme must be used within a ThemeProvider');
  }
  return context;
};