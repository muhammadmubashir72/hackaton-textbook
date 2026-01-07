/**
 * Component tests for TranslationButton
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import TranslationButton from '../../src/components/TranslationButton/TranslationButton';

// Mock the useTranslation hook
jest.mock('../../src/hooks/useTranslation', () => ({
  __esModule: true,
  default: jest.fn(() => ({
    isTranslating: false,
    isTranslated: false,
    translationError: null,
    toggleTranslation: jest.fn(),
    translateElement: jest.fn(),
    resetForNewChapter: jest.fn(),
    clearTranslationState: jest.fn(),
    loadTranslationState: jest.fn(),
    getLoadingState: jest.fn(),
    canTranslate: jest.fn(() => true),
    updateLastTranslationTime: jest.fn()
  }))
}));

import useTranslation from '../../src/hooks/useTranslation';

describe('TranslationButton', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders translation button with correct label', () => {
    render(<TranslationButton />);

    const button = screen.getByRole('button');
    expect(button).toBeInTheDocument();
    expect(button).toHaveAttribute('aria-label', 'Translate to Urdu');
    expect(button).toHaveTextContent('Urdu');
  });

  test('shows loading state when translating', () => {
    useTranslation.mockReturnValue({
      isTranslating: true,
      isTranslated: false,
      translationError: null,
      toggleTranslation: jest.fn(),
      translateElement: jest.fn(),
      resetForNewChapter: jest.fn(),
      clearTranslationState: jest.fn(),
      loadTranslationState: jest.fn(),
      getLoadingState: jest.fn(),
      canTranslate: jest.fn(() => true),
      updateLastTranslationTime: jest.fn()
    });

    render(<TranslationButton />);

    expect(screen.getByText('Translating...')).toBeInTheDocument();
    expect(screen.getByText('Translating...')).toBeInTheDocument();
  });

  test('shows correct label when content is translated', () => {
    useTranslation.mockReturnValue({
      isTranslating: false,
      isTranslated: true,
      translationError: null,
      toggleTranslation: jest.fn(),
      translateElement: jest.fn(),
      resetForNewChapter: jest.fn(),
      clearTranslationState: jest.fn(),
      loadTranslationState: jest.fn(),
      getLoadingState: jest.fn(),
      canTranslate: jest.fn(() => true),
      updateLastTranslationTime: jest.fn()
    });

    render(<TranslationButton />);

    const button = screen.getByRole('button');
    expect(button).toHaveAttribute('aria-label', 'Translate to English');
    expect(button).toHaveTextContent('English');
  });

  test('calls toggleTranslation when clicked', async () => {
    const mockToggleTranslation = jest.fn();
    useTranslation.mockReturnValue({
      isTranslating: false,
      isTranslated: false,
      translationError: null,
      toggleTranslation: mockToggleTranslation,
      translateElement: jest.fn(),
      resetForNewChapter: jest.fn(),
      clearTranslationState: jest.fn(),
      loadTranslationState: jest.fn(),
      getLoadingState: jest.fn(),
      canTranslate: jest.fn(() => true),
      updateLastTranslationTime: jest.fn()
    });

    render(<TranslationButton />);

    const button = screen.getByRole('button');
    fireEvent.click(button);

    await waitFor(() => {
      expect(mockToggleTranslation).toHaveBeenCalled();
    });
  });

  test('shows error message when translation error occurs', () => {
    useTranslation.mockReturnValue({
      isTranslating: false,
      isTranslated: false,
      translationError: 'Translation failed',
      toggleTranslation: jest.fn(),
      translateElement: jest.fn(),
      resetForNewChapter: jest.fn(),
      clearTranslationState: jest.fn(),
      loadTranslationState: jest.fn(),
      getLoadingState: jest.fn(),
      canTranslate: jest.fn(() => true),
      updateLastTranslationTime: jest.fn()
    });

    render(<TranslationButton />);

    expect(screen.getByText('Translation error: Translation failed')).toBeInTheDocument();
  });
});