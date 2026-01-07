# Data Model: Urdu Translation Toggle

## Translation State Entity

**Entity Name**: TranslationState
- **id**: string (chapter identifier)
- **isTranslated**: boolean (current translation status)
- **originalContent**: object (preserved original content structure)
- **translatedContent**: object (cached translated content structure)
- **lastUpdated**: timestamp (when translation was performed)

## Chapter Content Entity

**Entity Name**: ChapterContent
- **id**: string (unique chapter identifier)
- **title**: string (chapter title)
- **content**: string (HTML content of the chapter)
- **language**: string (original language, e.g., 'en')
- **translatedLanguage**: string (target language, e.g., 'ur')

## Validation Rules

1. TranslationState must have a valid chapter id
2. isTranslated must be a boolean value
3. originalContent and translatedContent must be valid objects
4. lastUpdated must be a valid timestamp
5. ChapterContent id must be unique

## State Transitions

- **Initial State**: {isTranslated: false}
- **After Translation**: {isTranslated: true}
- **After Toggle Back**: {isTranslated: false}
- **On Chapter Navigation**: Reset to {isTranslated: false}