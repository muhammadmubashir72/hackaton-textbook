import React, { useState, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

const CustomThemeToggle = () => {
  const { colorMode, setColorMode } = useColorMode();
  const [cycleIndex, setCycleIndex] = useState(0); // 0: system preference, 1: opposite

  // Determine system preference
  const getSystemPreference = () => {
    if (typeof window !== 'undefined') {
      return window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    }
    return 'light'; // default
  };

  const handleClick = () => {
    const systemPref = getSystemPreference();
    const oppositeMode = systemPref === 'light' ? 'dark' : 'light';

    if (cycleIndex === 0) {
      // Cycle to opposite mode (first press)
      setColorMode(oppositeMode);
      setCycleIndex(1);
    } else {
      // Cycle back to system preference (second press)
      setColorMode(systemPref);
      setCycleIndex(0);
    }
  };

  // Initialize cycle index based on current color mode
  useEffect(() => {
    const systemPref = getSystemPreference();
    if (colorMode === systemPref || colorMode === 'auto') {
      setCycleIndex(0); // System preference mode
    } else {
      setCycleIndex(1); // Opposite mode
    }
  }, [colorMode]);

  const systemPref = getSystemPreference();
  const isSystemMode = cycleIndex === 0;
  const currentMode = isSystemMode ? systemPref : (systemPref === 'light' ? 'dark' : 'light');
  const icon = currentMode === 'dark' ? '🌙' : '☀️';

  return (
    <button
      onClick={handleClick}
      className="navbar__item navbar__link custom-theme-toggle"
      aria-label={`Switch to ${isSystemMode ? (systemPref === 'dark' ? 'light' : 'dark') : systemPref} mode`}
      title={`Switch to ${isSystemMode ? (systemPref === 'dark' ? 'light' : 'dark') : systemPref} mode`}
      style={{
        fontSize: '1.2rem',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        width: '40px',
        height: '40px',
        border: 'none',
        backgroundColor: 'transparent',
        cursor: 'pointer',
        marginLeft: '0.5rem',
        borderRadius: '4px',
        transition: 'all 0.2s ease'
      }}
    >
      <span>{icon}</span>
    </button>
  );
};

export default CustomThemeToggle;