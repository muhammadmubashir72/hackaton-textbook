import React, { useCallback } from 'react';
import { Globe, Loader2, LetterText } from 'lucide-react';
import useTranslation from '../../hooks/useTranslation';
import styles from './TranslationButton.module.css';

const TranslationButton = ({ chapterId: propChapterId }) => {
  const {
    isTranslating,
    isTranslated,
    translationError,
    toggleTranslation,
    canTranslate,
    updateLastTranslationTime
  } = useTranslation(propChapterId);

  const handleClick = useCallback(async () => {
    // Check if we can translate (rate limiting)
    if (!canTranslate()) {
      console.warn('Translation rate limit exceeded. Please wait before translating again.');
      return;
    }

    // Find the main content element to translate
    const contentElement = document.querySelector('.markdown') || document.querySelector('.docItemContainer') || document.querySelector('main') || document.body;

    if (!contentElement) {
      console.error('No content element found for translation');
      return;
    }

    try {
      // Update the last translation time before starting
      updateLastTranslationTime();
      await toggleTranslation(contentElement);
    } catch (error) {
      console.error('Translation failed:', error);
    }
  }, [toggleTranslation, canTranslate, updateLastTranslationTime]);

  return (
    <div className={styles.translationButtonContainer}>
      <button
        onClick={handleClick}
        disabled={isTranslating}
        className={`${styles.translationButton} ${isTranslating ? styles.loading : ''}`}
        aria-label={isTranslated ? 'Switch to English' : 'Switch to اردو'}
        title={isTranslated ? 'Show in English' : 'Translate to اردو'}
      >
        {isTranslating ? (
          <>
            <Loader2 className={styles.icon} size={18} />
            Translating...
          </>
        ) : (
          <>
            <Globe className={styles.icon} size={18} />
            {isTranslated ? 'English' : 'اردو'}
          </>
        )}
      </button>
      {translationError && (
        <div className={styles.error}>
          Translation error: {translationError}
        </div>
      )}
    </div>
  );
};

export default TranslationButton;
