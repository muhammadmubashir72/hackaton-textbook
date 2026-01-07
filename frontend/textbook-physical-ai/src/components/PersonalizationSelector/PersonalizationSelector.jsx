import React, { useState, useEffect } from 'react';
import { BookOpen, Zap, Cpu, Loader2, ChevronDown, Check } from 'lucide-react';
import { personalizeContent } from '../../services/personalizationAPI';
import styles from './PersonalizationSelector.module.css';

const PersonalizationSelector = () => {
  const [selectedLevel, setSelectedLevel] = useState('medium');
  const [isOpen, setIsOpen] = useState(false);
  const [isAdapting, setIsAdapting] = useState(false);
  const [originalContent, setOriginalContent] = useState('');

  useEffect(() => {
    // Always default to medium for consistency with content level
    const savedLevel = 'medium';
    setSelectedLevel(savedLevel);

    // Store original content
    const contentEl = document.querySelector('.markdown');
    if (contentEl) {
      setOriginalContent(contentEl.innerHTML);
    }

    // Listen for content updates
    const observer = new MutationObserver(() => {
      const el = document.querySelector('.markdown');
      if (el && !isAdapting) {
        setOriginalContent(el.innerHTML);
      }
    });

    if (contentEl) {
      observer.observe(contentEl, { childList: true, subtree: true });
    }

    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    // Listen for level change events from other components
    const handleLevelChange = async (e) => {
      const { level } = e.detail;
      if (level !== selectedLevel) {
        await adaptContentToLevel(level);
      }
    };

    window.addEventListener('levelChange', handleLevelChange);
    return () => window.removeEventListener('levelChange', handleLevelChange);
  }, [selectedLevel, originalContent]);

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

  const adaptContentToLevel = async (newLevel) => {
    const contentElement = document.querySelector('.markdown');
    if (!contentElement) return;

    try {
      setIsAdapting(true);

      // Extract text content
      const textContent = contentElement.innerText;

      if (!textContent || textContent.trim().length === 0) {
        showNotification('No content to adapt');
        return;
      }

      // Call personalization API
      const response = await personalizeContent(textContent, newLevel);

      // Update page content with personalized version
      contentElement.innerHTML = response.personalized_content;

      // Update selected level
      setSelectedLevel(newLevel);
      localStorage.setItem('preferredLevel', newLevel);

      showNotification(`Content adapted to ${levels.find(l => l.value === newLevel)?.label} level`);
    } catch (error) {
      console.error('Personalization error:', error);
      showNotification('Failed to adapt content. Please try again.');
    } finally {
      setIsAdapting(false);
    }
  };

  const handleLevelChange = (level) => {
    if (level.value !== selectedLevel) {
      adaptContentToLevel(level.value);
    }
    setIsOpen(false);
  };

  const showNotification = (message) => {
    // Create a simple notification
    const notification = document.createElement('div');
    notification.className = styles.notification;
    notification.textContent = message;
    document.body.appendChild(notification);

    setTimeout(() => {
      notification.classList.add(styles.show);
    }, 10);

    setTimeout(() => {
      notification.classList.remove(styles.show);
      setTimeout(() => {
        document.body.removeChild(notification);
      }, 300);
    }, 2000);
  };

  const currentLevel = levels.find(l => l.value === selectedLevel);
  const CurrentIcon = currentLevel?.icon || Zap;

  return (
    <div className={styles.container}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`${styles.selectorButton} ${isAdapting ? styles.adapting : ''}`}
        disabled={isAdapting}
        aria-label="Select content level"
      >
        {isAdapting ? (
          <>
            <Loader2 className={styles.icon} size={18} />
            Adapting...
          </>
        ) : (
          <>
            <CurrentIcon className={styles.icon} size={18} />
            <span className={styles.label}>{currentLevel?.label}</span>
            <ChevronDown className={styles.arrowIcon} size={14} />
          </>
        )}
      </button>

      {isOpen && (
        <div className={styles.dropdown}>
          {levels.map((level) => {
            const LevelIcon = level.icon;
            return (
              <button
                key={level.value}
                onClick={() => handleLevelChange(level)}
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

      <style>{`
        @keyframes slideIn {
          from { opacity: 0; transform: translateY(-10px); }
          to { opacity: 1; transform: translateY(0); }
        }
      `}</style>
    </div>
  );
};

export default PersonalizationSelector;
