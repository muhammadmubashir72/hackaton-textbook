import React, { useState, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import styles from './index.module.css';

// Custom theme toggle that cycles between system preference and opposite mode with 2 clicks
const ThemeToggleCustom = (props) => {
  const { colorMode, setColorMode } = useColorMode();
  const [toggleState, setToggleState] = useState(0); // 0: system, 1: opposite

  // Determine system preference
  const getSystemPreference = () => {
    if (typeof window !== 'undefined') {
      return window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    }
    return 'light';
  };

  const handleClick = () => {
    const systemPref = getSystemPreference();
    const oppositeMode = systemPref === 'light' ? 'dark' : 'light';

    if (toggleState === 0) {
      // Switch to opposite mode
      setColorMode(oppositeMode);
      setToggleState(1);
    } else {
      // Switch back to system mode
      setColorMode(systemPref);
      setToggleState(0);
    }
  };

  // Initialize toggle state based on current color mode
  useEffect(() => {
    const systemPref = getSystemPreference();
    if (colorMode === systemPref || colorMode === 'auto') {
      setToggleState(0); // System mode
    } else {
      setToggleState(1); // Opposite mode
    }
  }, [colorMode]);

  const systemPref = getSystemPreference();
  const isSystemMode = toggleState === 0;
  const icon = isSystemMode ? (systemPref === 'dark' ? '🌙' : '☀️') : (systemPref === 'dark' ? '☀️' : '🌙');

  return (
    <button
      {...props}
      onClick={handleClick}
      className={`${styles.themeToggleCustom} ${props.className || ''}`}
      aria-label={isSystemMode ? `Switch to ${systemPref === 'dark' ? 'light' : 'dark'} mode` : `Switch to ${systemPref} (system) mode`}
      title={isSystemMode ? `Switch to ${systemPref === 'dark' ? 'light' : 'dark'} mode` : `Switch to ${systemPref} (system) mode`}
    >
      <span>{icon}</span>
    </button>
  );
};

export default ThemeToggleCustom;