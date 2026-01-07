# Meaning Lookup Skill

## Purpose
Generate contextual explanations for selected text in the interactive digital textbook, returning explanations in the currently active language via Next.js i18n locale management.

## Implementation
- Integration with LLM (GPT, Claude, etc.) for contextual explanations
- Next.js i18n locale validation for language consistency
- Context-aware processing to provide relevant meanings
- Caching mechanism for frequently requested explanations

## API
- `getMeaning(text, context, targetLang)`: Get explanation for selected text
- `getContextRelevance(text, context)`: Calculate relevance of context to selected text
- `formatExplanation(explanation, targetLang)`: Format explanation in target language
- `cacheExplanation(key, result)`: Store explanation in cache
- `getCachedExplanation(key)`: Retrieve cached explanation
- `validateTargetLanguage(targetLang)`: Validate target language against Next.js i18n config

## Parameters
- `text`: Text to get meaning for
- `context`: Surrounding context for better explanations
- `targetLang`: Target language for explanation ('en' or 'ur')
- `key`: Cache key for storing/retrieving explanation
- `result`: Explanation result to cache
- `targetLang`: Language code to validate

## Return Values
- `getMeaning`: Promise resolving to explanation object with meaning and metadata
- `getContextRelevance`: Number representing relevance score (0-1)
- `formatExplanation`: Formatted explanation string
- `cacheExplanation`: Boolean success status
- `getCachedExplanation`: Cached explanation result or null
- `validateTargetLanguage`: Boolean indicating if language is supported

## Error Handling
- Handle LLM API rate limiting and quotas
- Validate target language against Next.js i18n configuration
- Fallback to dictionary definitions if LLM unavailable
- Retry mechanism for failed explanation requests
- Graceful degradation when service unavailable
- Handle explanation errors and return appropriate fallback

## Dependencies
- Next.js i18n configuration
- LLM API service (GPT, Claude, etc.)
- Caching mechanism (localStorage or in-memory)
- Natural language processing utilities
- Dictionary API for fallback definitions