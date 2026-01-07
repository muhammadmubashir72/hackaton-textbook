import React, { useState, useEffect, useRef } from 'react';
import { Languages, BookOpen, Zap, Cpu, Loader2, ChevronDown, Check } from 'lucide-react';
import { personalizeContent } from '../../services/personalizationAPI';
import translationService from '../../services/translationService';
import { getTextNodesForTranslation, replaceTextContent } from '../../utils/domUtils';
import styles from './ChapterControls.module.css';

const ChapterControls = ({ chapterId, contentElementRef }) => {
  const [selectedLevel, setSelectedLevel] = useState('medium');
  const [translationState, setTranslationState] = useState('english'); // 'english' or 'urdu'
  const [isAdapting, setIsAdapting] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [personalizationDropdownOpen, setPersonalizationDropdownOpen] = useState(false);
  const originalContentRef = useRef(null); // For translation
  const originalMediumContentRef = useRef(null); // For personalization - stores original Medium-level content

  useEffect(() => {
    // Always default to medium for consistency with content level
    const savedLevel = 'medium';
    setSelectedLevel(savedLevel);

    // Always default to English for new chapters
    setTranslationState('english');

    // Store the original Medium-level content when component mounts
    if (contentElementRef?.current && !originalMediumContentRef.current) {
      originalMediumContentRef.current = contentElementRef.current.innerHTML;
    }
  }, [chapterId]);

  // Update translation state when content element is available
  useEffect(() => {
    if (contentElementRef?.current) {
      const element = contentElementRef.current;
      const currentContent = element.innerHTML.substring(0, 200); // Check first 200 chars
      const isCurrentlyUrdu = translationService.isUrduText(currentContent);

      // Set state based on actual content language
      setTranslationState(isCurrentlyUrdu ? 'urdu' : 'english');

      // Store the original Medium-level content if not already stored
      if (!originalMediumContentRef.current) {
        originalMediumContentRef.current = element.innerHTML;
      }
    }
  }, [contentElementRef?.current]);

  const levels = [
    {
      value: 'beginner',
      label: 'Easy',
      description: 'Simple explanations, easy to understand',
      icon: BookOpen
    },
    {
      value: 'medium',
      label: 'Medium',  // Changed from 'Intermediate' to 'Medium' as requested
      description: 'Moderate technical details, balanced content',
      icon: Zap
    },
    {
      value: 'advanced',
      label: 'Advanced',
      description: 'Deep technical content, advanced concepts',
      icon: Cpu
    }
  ];

  // Translation options removed since we're using a toggle button

  const handleLevelChange = async (newLevel) => {
    if (!contentElementRef?.current) {
      console.error('Content element ref not available');
      return;
    }

    try {
      setIsAdapting(true);

      const element = contentElementRef.current;

      // Special handling for Medium level - restore original content
      if (newLevel === 'medium') {
        if (originalMediumContentRef.current) {
          // Restore the original Medium-level content
          element.innerHTML = originalMediumContentRef.current;

          // Update selected level
          setSelectedLevel(newLevel);
          localStorage.setItem('preferredLevel', newLevel);

          // Reset translation state to English when restoring original
          setTranslationState('english');
          localStorage.setItem('preferredLanguage', 'english');
          originalContentRef.current = null; // Clear translation reference

          showNotification('Content restored to Medium level');
        } else {
          showNotification('No original content to restore');
        }
        return;
      }

      // For Easy and Advanced levels, use the original Medium content as base
      const baseContent = originalMediumContentRef.current || element.innerHTML;

      if (!baseContent || baseContent.trim().length === 0) {
        showNotification('No content to adapt');
        return;
      }

      console.log(`Adapting content to ${newLevel} level...`);
      console.log(`Content length being sent: ${baseContent.length} characters`);

      // Call personalization API with original Medium-level HTML content
      const response = await personalizeContent(baseContent, newLevel);

      if (!response || !response.personalized_content) {
        console.error('Invalid response from personalization API:', response);
        showNotification('Invalid response from server');
        return;
      }

      // Update content with personalized version
      element.innerHTML = response.personalized_content;

      // Update selected level
      setSelectedLevel(newLevel);
      localStorage.setItem('preferredLevel', newLevel);

      // Reset translation state to English when personalizing
      // (personalization is always done on English content)
      setTranslationState('english');
      localStorage.setItem('preferredLanguage', 'english');
      originalContentRef.current = null; // Clear translation reference

      showNotification(`Content adapted to ${levels.find(l => l.value === newLevel)?.label} level`);
    } catch (error) {
      console.error('Personalization error:', error);
      showNotification('Failed to adapt content. Please try again.');
    } finally {
      setIsAdapting(false);
      setPersonalizationDropdownOpen(false);
    }
  };

  const handleTranslationToggle = async () => {
    if (!contentElementRef?.current) return;

    try {
      setIsTranslating(true);

      const element = contentElementRef.current;

      // Use the current translation state instead of detecting from content
      // This prevents issues with incorrect language detection
      const isCurrentlyUrdu = translationState === 'urdu';

      console.log('Current translation state:', translationState);
      console.log('Is currently Urdu (from state):', isCurrentlyUrdu);

      if (isCurrentlyUrdu) {
        // Currently in Urdu (based on state), switch back to English
        // Restore original English content if stored
        if (originalContentRef.current) {
          // Restore the original English content (at current personalization level)
          element.innerHTML = originalContentRef.current;

          // Update the translation state to reflect the change
          setTranslationState('english');
          localStorage.setItem('preferredLanguage', 'english');
          showNotification('Content switched to English');

          // Clear the original content ref when switching back to English to avoid stale content
          originalContentRef.current = null;
        } else {
          showNotification('No original content to restore');
          return; // Exit without updating state
        }
      } else {
        // Currently in English (based on state), switch to Urdu
        // Store current English content before translating (might be personalized)
        const currentContent = element.innerHTML;
        originalContentRef.current = currentContent;

        // Extract text nodes for translation using the page translation API
        const textNodes = getTextNodesForTranslation(element);
        const originalTexts = textNodes.map(node => node.nodeValue).filter(text => text.trim() !== '');

        if (originalTexts.length > 0) {
          const apiUrl = window._env_?.REACT_APP_BACKEND_URL || process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001';
          const response = await fetch(`${apiUrl}/translate-page`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              texts: originalTexts,
              target_language: 'ur',
            }),
          });

          if (response.ok) {
            const data = await response.json();
            // Replace the translated texts back into the DOM
            for (let i = 0; i < textNodes.length && i < data.translated_texts.length; i++) {
              textNodes[i].nodeValue = data.translated_texts[i];
            }
            // Update the translation state to reflect the change
            setTranslationState('urdu');
            localStorage.setItem('preferredLanguage', 'urdu');
            showNotification('Content switched to اردو');
          } else {
            showNotification('Failed to translate to Urdu');
            // Restore original content on failure
            originalContentRef.current = null;
            return; // Exit without updating state
          }
        } else {
          showNotification('No text content found to translate');
          // Clear the stored content since we're not translating
          originalContentRef.current = null;
          return; // Exit without updating state
        }
      }

    } catch (error) {
      console.error('Translation error:', error);
      showNotification('Failed to translate content. Please try again.');
      // Restore original content on error
      if (originalContentRef.current && translationState === 'english') {
        originalContentRef.current = null;
      }
    } finally {
      setIsTranslating(false);
    }
  };

  const showNotification = (message) => {
    // Create a simple notification
    const notification = document.createElement('div');
    notification.style.cssText = `
      position: fixed;
      top: 20px;
      right: 20px;
      background: #333;
      color: white;
      padding: 12px 16px;
      border-radius: 4px;
      z-index: 10000;
      opacity: 0;
      transform: translateX(100%);
      transition: all 0.3s ease;
    `;
    notification.textContent = message;
    document.body.appendChild(notification);

    // Animate in
    setTimeout(() => {
      notification.style.cssText += `
        opacity: 1;
        transform: translateX(0);
      `;
    }, 10);

    // Remove after delay
    setTimeout(() => {
      notification.style.cssText += `
        opacity: 0;
        transform: translateX(100%);
      `;
      setTimeout(() => {
        document.body.removeChild(notification);
      }, 300);
    }, 2000);
  };

  const currentLevel = levels.find(l => l.value === selectedLevel);
  const CurrentLevelIcon = currentLevel?.icon || Zap;

  // Removed translation options since we're using a toggle button

  return (
    <div className={styles.container}>
      {/* Translation Control */}
      <div className={styles.controlGroup}>
        <button
          onClick={handleTranslationToggle}
          className={`${styles.controlButton} ${isTranslating ? styles.loading : ''}`}
          disabled={isTranslating || isAdapting}
          aria-label={`Switch to ${translationState === 'english' ? 'اردو' : 'English'}`}
          title={`Switch to ${translationState === 'english' ? 'اردو' : 'English'}`}
        >
          {isTranslating ? (
            <>
              <Loader2 className={`${styles.icon} ${styles.spinner}`} size={18} />
              Translating...
            </>
          ) : (
            <>
              <Languages className={styles.icon} size={18} />
              <span className={styles.label}>
                {translationState === 'english' ? 'اردو' : 'English'}
              </span>
            </>
          )}
        </button>
      </div>

      {/* Personalization Control */}
      <div className={styles.controlGroup}>
        <button
          onClick={() => setPersonalizationDropdownOpen(!personalizationDropdownOpen)}
          className={`${styles.controlButton} ${isAdapting ? styles.loading : ''}`}
          disabled={isTranslating || isAdapting}
          aria-label="Select content level"
        >
          {isAdapting ? (
            <>
              <Loader2 className={`${styles.icon} ${styles.spinner}`} size={18} />
              Adapting (30-60s)...
            </>
          ) : (
            <>
              <CurrentLevelIcon className={styles.icon} size={18} />
              <span className={styles.label}>{currentLevel?.label}</span>
              <ChevronDown className={styles.arrowIcon} size={14} />
            </>
          )}
        </button>

        {personalizationDropdownOpen && (
          <div className={styles.dropdown}>
            {levels.map((level) => {
              const LevelIcon = level.icon;
              return (
                <button
                  key={level.value}
                  onClick={() => handleLevelChange(level.value)}
                  className={`${styles.option} ${selectedLevel === level.value ? styles.active : ''}`}
                  disabled={isAdapting}
                >
                  <LevelIcon className={styles.optionIcon} size={24} />
                  <div className={styles.optionContent}>
                    <span className={styles.optionLabel}>{level.label}</span>
                    <span className={styles.optionDescription}>{level.description}</span>
                  </div>
                  {selectedLevel === level.value && (
                    <Check className={styles.checkmarkIcon} size={18} />
                  )}
                </button>
              );
            })}
          </div>
        )}
      </div>
    </div>
  );
};

export default ChapterControls;