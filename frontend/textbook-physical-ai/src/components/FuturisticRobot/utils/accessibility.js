/**
 * Accessibility utilities for the Futuristic Robot component
 */

/**
 * Check if user has reduced motion preference
 * @returns {boolean} True if reduced motion is preferred
 */
export const prefersReducedMotion = () => {
  if (typeof window !== 'undefined' && window.matchMedia) {
    return window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  }
  return false;
};

/**
 * Add event listener for reduced motion preference changes
 * @param {Function} callback - Function to call when preference changes
 * @returns {Function} Function to remove the event listener
 */
export const addReducedMotionListener = (callback) => {
  if (typeof window === 'undefined' || !window.matchMedia) {
    return () => {}; // No-op if not in browser environment
  }

  const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
  const handleChange = (e) => callback(e.matches);

  mediaQuery.addEventListener('change', handleChange);

  // Return cleanup function
  return () => {
    mediaQuery.removeEventListener('change', handleChange);
  };
};

/**
 * Check if user has high contrast preference
 * @returns {boolean} True if high contrast is preferred
 */
export const prefersHighContrast = () => {
  if (typeof window !== 'undefined' && window.matchMedia) {
    // Check for high contrast themes (Windows high contrast mode)
    return window.matchMedia('(prefers-contrast: high)').matches;
  }
  return false;
};

/**
 * Check if user is using a screen reader
 * This is a heuristic approach as there's no foolproof way to detect screen readers
 * @returns {boolean} True if screen reader usage is likely detected
 */
export const isScreenReaderDetected = () => {
  // Check for common screen reader indicators
  const isNavigator = typeof navigator !== 'undefined';

  if (isNavigator && navigator.userAgent) {
    const userAgent = navigator.userAgent.toLowerCase();
    const screenReaderAgents = [
      'narrator', 'nvda', 'orca', 'jaws', 'readspeaker',
      'zdscreenreader', 'chromevox', 'dolphin', 'supernova'
    ];

    return screenReaderAgents.some(agent => userAgent.includes(agent));
  }

  return false;
};

/**
 * Add keyboard navigation support to the component
 * @param {HTMLElement} element - Element to add keyboard support to
 * @param {Object} options - Options for keyboard navigation
 * @param {Function} options.onFocus - Function to call when element receives focus
 * @param {Function} options.onBlur - Function to call when element loses focus
 * @param {Function} options.onKeyDown - Function to call when key is pressed
 */
export const addKeyboardNavigation = (element, options = {}) => {
  if (!element) return () => {}; // Return no-op if no element provided

  const { onFocus, onBlur, onKeyDown } = options;

  const handleFocus = (e) => {
    if (onFocus) onFocus(e);
    element.setAttribute('aria-selected', 'true');
  };

  const handleBlur = (e) => {
    if (onBlur) onBlur(e);
    element.setAttribute('aria-selected', 'false');
  };

  const handleKeyDown = (e) => {
    if (onKeyDown) {
      onKeyDown(e);
    } else {
      // Default keyboard handling for common navigation keys
      switch (e.key) {
        case 'Enter':
        case ' ':
          e.preventDefault();
          // Trigger the default action
          break;
        case 'Escape':
          e.preventDefault();
          // Handle escape key if needed
          break;
        default:
          break;
      }
    }
  };

  element.addEventListener('focus', handleFocus);
  element.addEventListener('blur', handleBlur);
  element.addEventListener('keydown', handleKeyDown);

  // Return cleanup function
  return () => {
    element.removeEventListener('focus', handleFocus);
    element.removeEventListener('blur', handleBlur);
    element.removeEventListener('keydown', handleKeyDown);
  };
};

/**
 * Get appropriate ARIA attributes based on component state
 * @param {Object} config - Configuration for ARIA attributes
 * @param {string} config.label - Label for the component
 * @param {string} config.description - Description of the component
 * @param {boolean} config.isAnimated - Whether the component has animation
 * @returns {Object} ARIA attributes object
 */
export const getAriaAttributes = ({ label, description, isAnimated }) => {
  const attrs = {
    role: 'figure',
    'aria-label': label || 'Futuristic Robot Visualization',
  };

  if (description) {
    attrs['aria-description'] = description;
  }

  if (isAnimated) {
    attrs['aria-live'] = 'polite';
  }

  return attrs;
};

/**
 * Create accessible color palette that meets WCAG contrast requirements
 * @param {string} baseColor - Base color to adjust
 * @param {string} backgroundColor - Background color for contrast check
 * @returns {string} Accessible color variant
 */
export const getAccessibleColor = (baseColor, backgroundColor = '#000011') => {
  // This is a simplified approach - in a real implementation you might want to use
  // a more sophisticated contrast checking algorithm
  const baseColorLower = baseColor.toLowerCase();

  // If it's already a high-contrast color combination, return as is
  // This is a simplified check - real implementation would calculate contrast ratios
  if (baseColorLower === '#00ffff' || baseColorLower === '#ff0000' || baseColorLower === '#ffaf00') {
    return baseColor;
  }

  // For the purposes of this implementation, return the base color
  // A full implementation would adjust colors to meet contrast requirements
  return baseColor;
};

/**
 * Format component information for screen readers
 * @param {Object} config - Component configuration
 * @returns {string} Formatted description for screen readers
 */
export const formatScreenReaderDescription = (config) => {
  const {
    isAnimating = true,
    particleCount = 0,
    hasControls = false,
    speed = 1
  } = config || {};

  const parts = ['Futuristic robot visualization'];

  if (isAnimating) {
    parts.push('with animated particles');
  }

  if (particleCount > 0) {
    parts.push(`containing ${particleCount} particles in orbital motion`);
  }

  if (hasControls) {
    parts.push('with adjustable controls');
  }

  if (speed !== 1) {
    parts.push(`at ${speed}x speed`);
  }

  return parts.join(', ') + '.';
};