import React, { useState } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import styles from './InteractiveThemeToggle.module.css';

const InteractiveThemeToggle = ({ showSettings = false }) => {
  const {
    accentColor,
    fontSize,
    contrast,
    animations,
    interactiveElements,
    updateAccentColor,
    updateFontSize,
    updateContrast,
    toggleAnimations,
    toggleInteractiveElements
  } = useTheme();

  const [showThemeSettings, setShowThemeSettings] = useState(showSettings);

  const accentColors = [
    '#3578e5', // Blue
    '#ff6b6b', // Red
    '#4ecdc4', // Teal
    '#45b7d1', // Sky
    '#96ceb4', // Mint
    '#feca57', // Yellow
    '#ff9ff3', // Pink
    '#54a0ff', // Cornflower
  ];

  const fontSizes = [
    { value: 'small', label: 'Small' },
    { value: 'medium', label: 'Medium' },
    { value: 'large', label: 'Large' }
  ];

  const contrastOptions = [
    { value: 'normal', label: 'Normal' },
    { value: 'high', label: 'High Contrast' }
  ];

  const toggleSettings = () => {
    setShowThemeSettings(!showThemeSettings);
  };

  return (
    <div className={styles.themeToggleContainer}>
      <button
        className={`${styles.themeToggle} ${showThemeSettings ? styles.active : ''}`}
        onClick={toggleSettings}
        aria-label={showThemeSettings ? 'Hide theme settings' : 'Show theme settings'}
        title={showThemeSettings ? 'Hide theme settings' : 'Show theme settings'}
      >
        <span className={styles.themeIcon}>🎨</span>
        {showThemeSettings && <span className={styles.themeLabel}>Customize</span>}
      </button>

      {showThemeSettings && (
        <div className={styles.themeSettings}>
          <div className={styles.settingsSection}>
            <h4 className={styles.sectionTitle}>Accent Color</h4>
            <div className={styles.colorPalette}>
              {accentColors.map(color => (
                <button
                  key={color}
                  className={`${styles.colorSwatch} ${accentColor === color ? styles.active : ''}`}
                  style={{ backgroundColor: color }}
                  onClick={() => updateAccentColor(color)}
                  title={`Accent color: ${color}`}
                  aria-label={`Set accent color to ${color}`}
                />
              ))}
            </div>
          </div>

          <div className={styles.settingsSection}>
            <h4 className={styles.sectionTitle}>Text Size</h4>
            <div className={styles.fontSizeOptions}>
              {fontSizes.map(option => (
                <button
                  key={option.value}
                  className={`${styles.fontSizeButton} ${fontSize === option.value ? styles.active : ''}`}
                  onClick={() => updateFontSize(option.value)}
                >
                  {option.label}
                </button>
              ))}
            </div>
          </div>

          <div className={styles.settingsSection}>
            <h4 className={styles.sectionTitle}>Accessibility</h4>
            <div className={styles.accessibilityOptions}>
              <label className={styles.optionLabel}>
                <input
                  type="checkbox"
                  checked={contrast === 'high'}
                  onChange={(e) => updateContrast(e.target.checked ? 'high' : 'normal')}
                  className={styles.checkbox}
                />
                <span>High Contrast</span>
              </label>

              <label className={styles.optionLabel}>
                <input
                  type="checkbox"
                  checked={animations}
                  onChange={(e) => toggleAnimations()}
                  className={styles.checkbox}
                />
                <span>Animations</span>
              </label>

              <label className={styles.optionLabel}>
                <input
                  type="checkbox"
                  checked={interactiveElements}
                  onChange={(e) => toggleInteractiveElements()}
                  className={styles.checkbox}
                />
                <span>Interactive Elements</span>
              </label>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default InteractiveThemeToggle;