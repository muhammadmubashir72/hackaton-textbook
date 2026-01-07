# Translation Processing Skill

## Purpose
Translate text between English and Urdu for the interactive digital textbook's contextual translation feature, using Next.js i18n for locale management.

## Implementation
- Integration with translation API (Google Translate, Azure Translator, etc.)
- Caching mechanism for frequent translations
- Next.js i18n locale validation
- Error handling and fallback mechanisms

## API
- `translateText(text, sourceLang, targetLang)`: Translate text between languages
- `getSupportedLocales()`: Get list of supported locales from Next.js i18n config
- `isTranslationAvailable(text)`: Check if translation service is available
- `cacheTranslation(key, result)`: Store translation in cache
- `getCachedTranslation(key)`: Retrieve cached translation
- `validateLocale(locale)`: Validate locale against Next.js i18n configuration

## Parameters
- `text`: Text to translate
- `sourceLang`: Source language code ('en' or 'ur')
- `targetLang`: Target language code ('en' or 'ur')
- `key`: Cache key for storing/retrieving translation
- `result`: Translation result to cache
- `locale`: Locale code to validate

## Return Values
- `translateText`: Promise resolving to translation object with translated text and metadata
- `getSupportedLocales`: Array of supported locale codes from Next.js config
- `isTranslationAvailable`: Boolean indicating service availability
- `cacheTranslation`: Boolean success status
- `getCachedTranslation`: Cached translation result or null
- `validateLocale`: Boolean indicating if locale is valid

## Error Handling
- Handle API rate limiting and quotas
- Validate locales against Next.js i18n configuration
- Fallback to default locale if invalid
- Retry mechanism for failed translations
- Graceful degradation when service unavailable
- Handle translation errors and return original text

## Dependencies
- Next.js i18n configuration
- Translation API service (Google Translate, Azure Translator, etc.)
- Caching mechanism (localStorage or in-memory)
- Network request management