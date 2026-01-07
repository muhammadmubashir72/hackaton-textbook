# Research: Urdu Translation Implementation

## Decision: Implement Qdrant-based caching with Gemini LLM translation
**Rationale**: The existing architecture already uses Qdrant for vector storage and Gemini for LLM responses. By extending the Qdrant metadata to store translation pairs, we can efficiently cache translations and avoid repeated API calls to Gemini. This approach leverages existing infrastructure while providing performance benefits.

## Decision: Frontend integration with existing text selection buttons
**Rationale**: The system already has text selection buttons with an "Ask" feature. Adding an "Urdu" button follows the same pattern and maintains UI consistency. The existing event system can be extended to handle translation requests.

## Decision: Urdu detection using character range analysis
**Rationale**: Rather than using complex language detection libraries, we can detect Urdu text by checking if characters fall within the Arabic/Urdu Unicode range (U+0600 to U+06FF). This is lightweight and efficient for the use case.

## Decision: Backend translation endpoint with caching layer
**Rationale**: The existing FastAPI backend can be extended with a new translation endpoint that first checks Qdrant metadata for cached translations, then falls back to Gemini for new translations. This maintains separation of concerns while extending functionality.

## Decision: Chat interface reuse for translation results
**Rationale**: The existing chat interface already handles displaying responses from the backend. By routing translation results through the same display mechanism, we maintain UI consistency and reduce code duplication.

## Alternatives considered:
- Using a separate translation service (Google Translate API) - rejected because it would require additional API keys and doesn't leverage existing Qdrant caching
- Client-side translation libraries - rejected because they don't provide the same quality as LLM-based translation and don't support caching
- Separate UI component for translations - rejected because it would create inconsistency with the existing "Ask" feature