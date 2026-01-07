# Data Model: Urdu Translation

## Entities

### Translation Cache Entry
**Purpose**: Store cached translations in Qdrant metadata for fast retrieval

**Fields**:
- `original_text` (string): The original English text that was translated
- `urdu_translation` (string): The Urdu translation of the original text
- `created_at` (timestamp): When the translation was created
- `last_accessed` (timestamp): When the translation was last retrieved
- `access_count` (integer): Number of times this translation has been accessed

**Validation Rules**:
- `original_text` must not be empty
- `urdu_translation` must not be empty
- `created_at` is auto-generated on creation
- `last_accessed` is updated on each access

### Translation Request
**Purpose**: Request payload for translation API endpoint

**Fields**:
- `text` (string, required): The text to translate
- `target_language` (string, default: "ur"): Target language code

**Validation Rules**:
- `text` must be between 1 and 5000 characters
- `target_language` must be a valid language code (currently only "ur" supported)

### Translation Response
**Purpose**: Response payload from translation API endpoint

**Fields**:
- `original_text` (string): The original text that was translated
- `translated_text` (string): The translated text
- `target_language` (string): The target language code
- `cached` (boolean): Whether the result was retrieved from cache
- `timestamp` (timestamp): When the translation was completed

**Validation Rules**:
- `original_text` must match the input text
- `translated_text` must not be empty
- `cached` indicates if result came from cache or was generated

## Relationships

The Translation Cache Entry is stored as metadata in Qdrant vectors. Each cached translation is associated with a vector embedding of the original text for potential semantic search capabilities, though the primary lookup will be by exact text matching.

## State Transitions

Translation requests follow this flow:
1. Request received → Check cache
2. If cached → Return cached result and update access stats
3. If not cached → Generate translation via Gemini → Store in cache → Return result