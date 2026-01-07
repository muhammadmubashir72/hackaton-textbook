/**
 * Analytics service for tracking translation usage
 */

class AnalyticsService {
  /**
   * Track translation event
   * @param {string} action - The action being tracked (e.g., 'translate_to_urdu', 'translate_to_english')
   * @param {string} chapterId - The chapter ID where translation occurred
   * @param {number} duration - Time taken for translation in milliseconds
   * @param {boolean} success - Whether the translation was successful
   */
  trackTranslationEvent(action, chapterId, duration = null, success = true) {
    try {
      const event = {
        action,
        chapterId,
        timestamp: new Date().toISOString(),
        duration,
        success,
        userAgent: navigator.userAgent,
        language: navigator.language
      };

      // For now, log the event to console
      // In a real implementation, this would send to an analytics service
      console.log('Translation event tracked:', event);

      // Store in localStorage for later batch processing
      const events = JSON.parse(localStorage.getItem('translationEvents') || '[]');
      events.push(event);

      // Limit to 100 events to prevent localStorage from growing too large
      if (events.length > 100) {
        events.shift(); // Remove oldest event
      }

      localStorage.setItem('translationEvents', JSON.stringify(events));
    } catch (error) {
      console.warn('Failed to track translation event:', error);
    }
  }

  /**
   * Get tracked events
   * @returns {Array} - Array of tracked events
   */
  getTrackedEvents() {
    try {
      return JSON.parse(localStorage.getItem('translationEvents') || '[]');
    } catch (error) {
      console.warn('Failed to retrieve tracked events:', error);
      return [];
    }
  }

  /**
   * Clear tracked events
   */
  clearTrackedEvents() {
    try {
      localStorage.removeItem('translationEvents');
    } catch (error) {
      console.warn('Failed to clear tracked events:', error);
    }
  }
}

export default new AnalyticsService();