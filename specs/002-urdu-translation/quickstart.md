# Quickstart: Urdu Translation Feature

## Overview
This guide explains how to implement the Urdu translation feature that allows users to select text in Docusaurus chapter pages and translate it to Urdu using a button click. The system leverages Qdrant for caching translations and Gemini LLM for new translations.

## Prerequisites
- Node.js 18+ for frontend
- Python 3.11+ for backend
- Qdrant vector database running
- Cohere API key for embeddings
- Google Gemini API key for translations

## Architecture

### Frontend Components
1. **TextSelectionButtons**: Enhanced with "Urdu" button that captures selected text
2. **ChatKit**: Displays translation results using existing chat interface

### Backend Components
1. **Translation API**: New `/translate` endpoint in FastAPI
2. **Qdrant Integration**: Check for cached translations in metadata
3. **Gemini Integration**: Generate new translations when not cached

## Implementation Steps

### 1. Frontend Implementation
- Update TextSelectionButtons component to dispatch "urdu" translation events
- Add Urdu translation handler to ChatKit component
- Implement fetch call to backend translation endpoint

### 2. Backend Implementation
- Create new `/translate` endpoint in api_main.py
- Extend retrieval_service.py to handle translation caching
- Implement Urdu detection logic
- Add Qdrant metadata storage for cached translations

### 3. Integration Points
- Text selection → Backend API call → Qdrant cache check → Gemini translation → Result display
- Maintain existing "Ask" functionality alongside new "Urdu" functionality

## API Contract

### Translation Endpoint
```
POST /translate
Request: { "text": "string", "target_language": "ur" }
Response: { "original_text": "string", "translated_text": "string", "target_language": "ur", "cached": boolean }
```

## Testing
- Verify text selection captures correctly
- Test Urdu detection functionality
- Validate cache hit/miss behavior
- Confirm chat interface displays results properly