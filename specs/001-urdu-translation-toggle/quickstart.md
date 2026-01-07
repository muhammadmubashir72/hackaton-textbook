# Quickstart Guide: Urdu Translation Toggle

## Development Setup

1. Ensure you have the frontend environment set up with Docusaurus
2. Install required dependencies:
   ```bash
   npm install googletrans
   ```

## Key Components

### TranslationButton Component
- Located at: `frontend/src/components/TranslationButton/TranslationButton.js`
- Provides the UI element for the translation toggle
- Manages the button state (translated vs original)

### Translation Service
- Located at: `frontend/src/services/translationService.js`
- Handles the actual translation using googletrans
- Manages translation caching

### useTranslation Hook
- Located at: `frontend/src/hooks/useTranslation.js`
- Manages translation state and chapter-specific storage
- Provides translation and toggle functions

## Implementation Steps

1. Add the TranslationButton to chapter pages
2. Implement content traversal to identify text elements
3. Create translation service with googletrans integration
4. Implement state management for translation persistence
5. Add toggle functionality
6. Test preservation of code blocks, links, and formatting

## Testing

Run tests to ensure:
- Translation works for all text content
- Code blocks and links remain unchanged
- Toggle functionality works correctly
- Chapter-specific state is maintained
- Performance meets requirements (under 3 seconds)