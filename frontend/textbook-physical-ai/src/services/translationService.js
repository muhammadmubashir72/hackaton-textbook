/**
 * Translation service for handling Urdu translation functionality
 * Uses backend API for translation to comply with constitution requirements
 */
class TranslationService {
  /**
   * Translate text to Urdu using backend API
   * @param {string} text - The text to translate
   * @returns {Promise<string>} - The translated text
   */
  async translateToUrdu(text) {
    try {
      if (!text || typeof text !== 'string') {
        throw new Error('Invalid text provided for translation');
      }

      // Check if text is already in Urdu before translating
      if (this.isUrduText(text)) {
        return text; // Return as-is if already in Urdu
      }

      // Call the backend translation API
      const apiUrl = window._env_?.REACT_APP_BACKEND_URL || process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: text,
          target_language: 'ur',
        }),
      });

      if (!response.ok) {
        throw new Error(`Translation API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      return data.translated_text;
    } catch (error) {
      console.error('Translation error:', error);

      // Fallback: return the original text when translation fails
      // This ensures the original content is preserved when API is unavailable
      console.warn('Translation service unavailable, returning original text');
      return text; // Return original text instead of placeholder
    }
  }

  /**
   * Translate text from Urdu back to English using backend API
   * @param {string} text - The Urdu text to translate
   * @returns {Promise<string>} - The translated text
   */
  async translateFromUrdu(text) {
    try {
      if (!text || typeof text !== 'string') {
        throw new Error('Invalid text provided for translation');
      }

      // Call the backend translation API
      const apiUrl = window._env_?.REACT_APP_BACKEND_URL || process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001';
      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: text,
          target_language: 'en',
        }),
      });

      if (!response.ok) {
        throw new Error(`Translation API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      return data.translated_text;
    } catch (error) {
      console.error('Translation error:', error);

      // Fallback: return the original text when translation fails
      // This ensures the original content is preserved when API is unavailable
      console.warn('Translation service unavailable, returning original text');
      return text; // Return original text instead of placeholder
    }
  }

  /**
   * Check if text is already in Urdu
   * @param {string} text - The text to check
   * @returns {boolean} - True if text is in Urdu, false otherwise
   */
  isUrduText(text) {
    if (!text || typeof text !== 'string') {
      return false;
    }

    // Count Arabic/Urdu characters (Unicode range U+0600 to U+06FF)
    let arabicUrduChars = 0;
    let totalChars = 0;

    for (const char of text) {
      if (char >= '\u0600' && char <= '\u06FF') {
        arabicUrduChars++;
      }
      if (char.trim() && char.match(/[a-zA-Z\u0600-\u06FF]/)) {
        totalChars++;
      }
    }

    if (totalChars === 0) {
      return false;
    }

    // If more than 30% of characters are Arabic/Urdu, consider it Urdu
    return (arabicUrduChars / totalChars) > 0.3;
  }
}

// Export singleton instance
export default new TranslationService();