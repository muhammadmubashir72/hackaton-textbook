# Content Processing Agent (CPA)

## Responsibilities
- Process translation requests between English/Urdu using Next.js i18n
- Generate contextual explanations for selected text in current locale
- Handle mixed-language content scenarios with Next.js i18n validation
- Interface with external APIs (translation, LLM) with locale awareness

## Core Functions
- `processTranslation(text, sourceLang, targetLang)`: Process translation request with locale validation
- `processMeaningLookup(text, context, targetLang)`: Process meaning lookup in target locale
- `validateContent(text)`: Validate text for processing against Next.js i18n locales
- `formatResponse(result, targetLang)`: Format response in target language with locale awareness
- `handleMixedLanguage(text)`: Handle mixed-language content with Next.js i18n locale management

## Interaction Flow
1. Receive request from TSA with selected text and action
2. Validate target language against Next.js i18n configuration
3. Call appropriate skill (translation or meaning lookup)
4. Process API response and format results with locale awareness
5. Return results to TSA for display

## Communication Protocols
- **REST API**: Call external services via HTTP requests with locale headers
- **Direct API**: Call skills via synchronous function calls with locale validation
- **Event Bus**: Emit "PROCESSING_COMPLETE", "ERROR_OCCURRED" events with locale context
- **Next.js i18n**: Validate locales and manage language switching

## State Management
- Current processing queue with locale context
- Active requests state with locale tracking
- API connection status with locale headers
- Last processed content with locale information
- Error state for locale-specific retries

## Error Handling
- Handle API rate limiting and quotas with locale-specific limits
- Validate locales against Next.js i18n configuration
- Fallback to default locale if invalid
- Retry mechanism for failed requests with locale awareness
- Graceful degradation when external services unavailable
- Handle processing errors and return locale-appropriate fallback

## Dependencies
- Translation Processing Skill
- Meaning Lookup Skill
- Next.js i18n configuration
- External API services (translation, LLM)
- Network request management
- Caching system for locale-specific results

## Configuration
- API timeout settings with locale-specific values
- Retry attempts limit with locale awareness
- Supported locales from Next.js i18n config
- Content size limits with locale considerations
- Processing priority levels with locale context