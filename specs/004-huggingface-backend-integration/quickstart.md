# Quickstart Guide: Hugging Face Backend Integration

## Overview
This guide explains how to deploy and connect the ChatKit component to the Hugging Face backend with Qdrant, Cohere, and Google Gemini integration.

## Backend Deployment

### 1. Environment Variables Setup
```bash
# In your Hugging Face Space secrets or environment
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key
GEMINI_API_KEY=your-google-gemini-api-key
COLLECTION_NAME=your-qdrant-collection-name
EMBEDDING_MODEL=embed-english-v3.0
GEMINI_MODEL=gemini-pro
```

### 2. Hugging Face Deployment Structure
```
app/
├── __init__.py
├── agent.py
├── api_main.py          # Main FastAPI application
├── config.py           # Configuration and settings
├── main.py             # Hugging Face entry point
├── retrieving.py       # Legacy retrieval functions
└── services/
    └── retrieval_service.py  # Qdrant/Cohere integration
```

### 3. Hugging Face Main Entry Point
```python
# backend/main.py
import os
import uvicorn
from app.api_main import app

if __name__ == "__main__":
    uvicorn.run(
        "app.api_main:app",
        host="0.0.0.0",
        port=int(os.environ.get("PORT", 7860)),  # Hugging Face standard port
        reload=False,  # Disable reload for production
        log_level="info"
    )
```

## Frontend Integration

### 1. Update Backend Connection in ChatKit
```javascript
// In ChatKit component
const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'https://your-username-hf-space-name.hf.space';
const response = await fetch(`${BACKEND_URL}/query`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ query: message, top_k: 3 })
});
```

### 2. Environment Configuration for Frontend
```bash
# In your frontend environment (.env file)
REACT_APP_BACKEND_URL=https://your-username-hf-space-name.hf.space
```

## API Endpoints

### Primary Endpoint
- `POST /query` - Main endpoint for RAG queries
- Request: `{ query: string, top_k: number }`
- Response: `{ query: string, answer: string, sources: string[] }`

### Health Check Endpoints
- `GET /` - API status check
- `GET /health` - Overall health status
- `GET /health/qdrant` - Qdrant connectivity check
- `GET /health/gemini` - Google Gemini connectivity check

## Key Implementation Points

### Backend Configuration
- Configure CORS middleware for frontend communication
- Set up proper error handling for production use
- Optimize for Hugging Face resource constraints
- Implement health check endpoints for monitoring

### Frontend Connection
- Use environment-based backend URL configuration
- Implement proper error handling for network issues
- Add loading states for user experience
- Handle backend unavailability gracefully

### Content Boundary Enforcement
- All responses must be generated from Qdrant-stored book content
- Implement clear "not found" responses when information is unavailable
- Prevent any external knowledge usage in responses

## Testing the Integration

1. Verify Hugging Face backend is accessible at the configured URL
2. Test the `/health` endpoint to confirm all services are running
3. Verify that queries return results from the Qdrant database
4. Confirm that responses are based only on book content
5. Test the "not found" scenario with queries outside book scope
6. Validate that no external information is included in responses