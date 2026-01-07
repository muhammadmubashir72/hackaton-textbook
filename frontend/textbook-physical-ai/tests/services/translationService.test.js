/**
 * Unit tests for translationService
 * Note: These are basic structural tests since actual translation requires network calls
 */

// Mock the google-translate-api-browser
jest.mock('google-translate-api-browser', () => ({
  translate: jest.fn()
}));

import { translate } from 'google-translate-api-browser';
import TranslationService from '../../src/services/translationService';

describe('TranslationService', () => {
  let service;

  beforeEach(() => {
    service = new TranslationService();
    jest.clearAllMocks();
  });

  describe('translateToUrdu', () => {
    test('should translate English text to Urdu', async () => {
      // Mock the translation response
      const mockResult = { text: 'اردو متن' };
      translate.mockResolvedValue(mockResult);

      const result = await service.translateToUrdu('English text');

      expect(translate).toHaveBeenCalledWith('English text', { to: 'ur' });
      expect(result).toBe('اردو متن');
    });

    test('should handle invalid input', async () => {
      await expect(service.translateToUrdu(null)).rejects.toThrow('Invalid text provided for translation');
      await expect(service.translateToUrdu('')).rejects.toThrow('Invalid text provided for translation');
      await expect(service.translateToUrdu(123)).rejects.toThrow('Invalid text provided for translation');
    });

    test('should handle translation errors', async () => {
      translate.mockRejectedValue(new Error('Translation failed'));

      await expect(service.translateToUrdu('test')).rejects.toThrow('Translation failed');
    });
  });

  describe('translateFromUrdu', () => {
    test('should translate Urdu text to English', async () => {
      // Mock the translation response
      const mockResult = { text: 'English text' };
      translate.mockResolvedValue(mockResult);

      const result = await service.translateFromUrdu('اردو متن');

      expect(translate).toHaveBeenCalledWith('اردو متن', { to: 'en' });
      expect(result).toBe('English text');
    });

    test('should handle invalid input', async () => {
      await expect(service.translateFromUrdu(null)).rejects.toThrow('Invalid text provided for translation');
      await expect(service.translateFromUrdu('')).rejects.toThrow('Invalid text provided for translation');
      await expect(service.translateFromUrdu(123)).rejects.toThrow('Invalid text provided for translation');
    });
  });

  describe('isUrduText', () => {
    test('should return true for Urdu text', () => {
      const urduText = 'یہ اردو میں ہے';
      expect(service.isUrduText(urduText)).toBe(true);
    });

    test('should return false for English text', () => {
      const englishText = 'This is English text';
      expect(service.isUrduText(englishText)).toBe(false);
    });

    test('should return true for mixed text with significant Urdu content', () => {
      const mixedText = 'Hello یہ اردو ہے world';
      expect(service.isUrduText(mixedText)).toBe(true);
    });

    test('should return false for empty text', () => {
      expect(service.isUrduText('')).toBe(false);
      expect(service.isUrduText(null)).toBe(false);
      expect(service.isUrduText(undefined)).toBe(false);
    });
  });
});