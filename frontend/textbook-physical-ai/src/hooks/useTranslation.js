import { useState, useEffect, useCallback } from 'react';
import translationService from '../services/translationService';
import { getTextNodesForTranslation, extractOriginalContent, replaceTextContent } from '../utils/domUtils';
import analyticsService from '../utils/analytics';

/**
 * Custom hook for managing translation state and functionality
 */
const useTranslation = (chapterId) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [translationError, setTranslationError] = useState(null);
  const [originalContent, setOriginalContent] = useState(null);

  // Generate chapter-specific storage key
  const getStorageKey = useCallback((chapterId, suffix) => {
    return `translation_${chapterId}_${suffix}`;
  }, []);

  // Save translation state to localStorage
  const saveTranslationState = useCallback((chapterId, state) => {
    if (chapterId) {
      const key = getStorageKey(chapterId, 'state');
      localStorage.setItem(key, JSON.stringify(state));
    }
  }, [getStorageKey]);

  // Load translation state from localStorage
  const loadTranslationState = useCallback((chapterId) => {
    if (!chapterId) return null;

    const key = getStorageKey(chapterId, 'state');
    const savedState = localStorage.getItem(key);

    if (savedState) {
      try {
        return JSON.parse(savedState);
      } catch (error) {
        console.error('Error parsing saved translation state:', error);
        return null;
      }
    }
    return null;
  }, [getStorageKey]);

  // Get current chapter ID if not provided
  const getCurrentChapterId = useCallback(() => {
    if (chapterId) return chapterId;

    // Try to extract chapter ID from URL or page structure
    const path = window.location.pathname;
    const chapterMatch = path.match(/\/(docs\/[^\/]+|[^\/]+)$/);
    if (chapterMatch) {
      return chapterMatch[1].replace(/\//g, '_');
    }

    // Fallback to page title or URL hash
    return window.location.hash || document.title || 'default';
  }, [chapterId]);

  // Initialize translation state for current chapter
  useEffect(() => {
    const currentChapterId = getCurrentChapterId();
    const savedState = loadTranslationState(currentChapterId);

    if (savedState) {
      // Check if actual content matches the saved state
      const contentElement = document.querySelector('.markdown') || document.querySelector('.docItemContainer') || document.querySelector('main');
      if (contentElement) {
        const textNodes = getTextNodesForTranslation(contentElement);
        const isActuallyUrdu = textNodes.length > 0 && translationService.isUrduText(textNodes[0].nodeValue);

        // If saved state doesn't match actual content, correct it
        if (savedState.isTranslated && !isActuallyUrdu) {
          // Saved state says Urdu but content is English - clear the state
          setIsTranslated(false);
          setOriginalContent(null);
        } else if (!savedState.isTranslated && isActuallyUrdu) {
          // Saved state says English but content is Urdu - update state
          setIsTranslated(true);
          setOriginalContent(savedState.originalContent || null);
        } else {
          // State matches actual content
          setIsTranslated(savedState.isTranslated);
          setOriginalContent(savedState.originalContent || null);
        }
      } else {
        // No content element found, use saved state
        setIsTranslated(savedState.isTranslated);
        setOriginalContent(savedState.originalContent || null);
      }
    }
  }, [chapterId, getCurrentChapterId, loadTranslationState, translationService]);

  // Translate content in a specific element
  const translateElement = useCallback(async (element) => {
    if (!element) {
      throw new Error('No element provided for translation');
    }

    setIsTranslating(true);
    setTranslationError(null);
    const startTime = Date.now();

    try {
      // Get text nodes that need translation
      const textNodes = getTextNodesForTranslation(element);

      if (textNodes.length === 0) {
        console.warn('No translatable text found in element');
        return;
      }

      // Extract original text content
      const originalTexts = textNodes.map(node => node.nodeValue);

      // Determine if we're translating to or from Urdu
      const isCurrentlyUrdu = translationService.isUrduText(originalTexts.join(' '));
      let action;

      if (isCurrentlyUrdu && originalContent) {
        // Restore original English content (not translate from Urdu)
        element.innerHTML = originalContent;
        setOriginalContent(null); // Clear the stored original content
        action = 'restore_english';
      } else if (!isCurrentlyUrdu) {
        // Translate from English to Urdu
        // Store original content before translating
        const currentHTML = element.innerHTML;
        if (!originalContent) {
          setOriginalContent(currentHTML);
        }

        const translatedTexts = await Promise.all(
          originalTexts.map(text => translationService.translateToUrdu(text))
        );
        // Replace content with translated text
        replaceTextContent(element, originalTexts, translatedTexts);
        action = 'translate_to_urdu';
      } else {
        // Content is Urdu but no original content to restore
        console.warn('Content is in Urdu but no original content to restore');
        return;
      }

      // Calculate duration and track the event
      const duration = Date.now() - startTime;
      const currentChapterId = getCurrentChapterId();
      analyticsService.trackTranslationEvent(action, currentChapterId, duration, true);

    } catch (error) {
      console.error('Translation error:', error);
      setTranslationError(error.message);

      // Track failed translation event
      const currentChapterId = getCurrentChapterId();
      const duration = Date.now() - startTime;
      analyticsService.trackTranslationEvent('translate_error', currentChapterId, duration, false);

      throw error;
    } finally {
      setIsTranslating(false);
    }
  }, [originalContent, translationService, getCurrentChapterId]);

  // Toggle translation for a specific element
  const toggleTranslation = useCallback(async (element) => {
    if (isTranslating) {
      return; // Prevent multiple simultaneous translations
    }

    const currentChapterId = getCurrentChapterId();
    const isCurrentlyTranslated = isTranslated;

    try {
      // Translate the content
      await translateElement(element);

      // Check if content is now in Urdu after the operation
      const textNodes = getTextNodesForTranslation(element);
      const isNowUrdu = textNodes.length > 0 && translationService.isUrduText(textNodes[0].nodeValue);

      // Update the translated state based on actual content
      const newTranslatedState = isNowUrdu;
      setIsTranslated(newTranslatedState);

      // Save the new state
      saveTranslationState(currentChapterId, {
        isTranslated: newTranslatedState,
        originalContent: originalContent || element.innerHTML,
        lastUpdated: Date.now()
      });

    } catch (error) {
      console.error('Toggle translation error:', error);
      // Don't change the state if translation failed
    }
  }, [isTranslating, isTranslated, getCurrentChapterId, translateElement, saveTranslationState, originalContent, translationService]);

  // Get loading state for display
  const getLoadingState = useCallback(() => {
    return isTranslating;
  }, [isTranslating]);

  // Check for rate limiting (simple implementation)
  const canTranslate = useCallback(() => {
    const lastTranslationTime = localStorage.getItem('lastTranslationTime');
    const now = Date.now();

    // Simple rate limiting: prevent translations within 1 second of each other
    if (lastTranslationTime && (now - parseInt(lastTranslationTime)) < 1000) {
      return false;
    }
    return true;
  }, []);

  // Update last translation time
  const updateLastTranslationTime = useCallback(() => {
    localStorage.setItem('lastTranslationTime', Date.now().toString());
  }, []);

  // Reset translation state when chapter changes
  const resetForNewChapter = useCallback((newChapterId) => {
    const savedState = loadTranslationState(newChapterId);

    if (savedState) {
      setIsTranslated(savedState.isTranslated);
      setOriginalContent(savedState.originalContent || null);
    } else {
      setIsTranslated(false);
      setOriginalContent(null);
    }
  }, [loadTranslationState]);

  // Clear translation state for a chapter
  const clearTranslationState = useCallback((chapterIdToClear) => {
    const idToClear = chapterIdToClear || getCurrentChapterId();
    const key = getStorageKey(idToClear, 'state');
    localStorage.removeItem(key);

    if (idToClear === getCurrentChapterId()) {
      setIsTranslated(false);
      setOriginalContent(null);
    }
  }, [getCurrentChapterId, getStorageKey]);

  return {
    isTranslating,
    isTranslated,
    translationError,
    toggleTranslation,
    translateElement,
    resetForNewChapter,
    clearTranslationState,
    loadTranslationState,
    getLoadingState,
    canTranslate,
    updateLastTranslationTime
  };
};

export default useTranslation;