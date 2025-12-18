# RAG Chatbot System Overview

This document provides a comprehensive overview of the Retrieval-Augmented Generation (RAG) chatbot system for the Physical AI & Humanoid Robotics textbook.

## Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Docusaurus    │    │   FastAPI API    │    │  External APIs  │
│   Frontend      │    │   Backend        │    │                 │
│                 │    │                  │    │  ┌─────────────┐ │
│  ┌───────────┐  │    │  ┌─────────────┐ │    │  │  Google     │ │
│  │ChatInterface│◄┼────┼──►api_main.py  │◄┼────┼──►  Gemini   │ │
│  └───────────┘  │    │  │             │ │    │  └─────────────┘ │
│                 │    │  │services/    │ │    │  ┌─────────────┐ │
│                 │    │  │retrieval_   │◄┼────┼──►  Cohere     │ │
│                 │    │  │service.py   │ │    │  │  Embeddings │ │
│                 │    │  └─────────────┘ │    │  └─────────────┘ │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │  Vector Database │
                       │                  │
                       │  ┌─────────────┐ │
                       │  │   Qdrant    │ │
                       │  │   Cloud     │ │
                       │  └─────────────┘ │
                       └──────────────────┘
```

## System Components

### 1. Backend Services (Python/FastAPI)

**Main API (`app/api_main.py`)**
- FastAPI application serving the RAG chatbot endpoints
- Handles query requests and response generation
- Integrates with retrieval service and Gemini API

**Configuration (`app/config.py`)**
- Centralized configuration using Pydantic Settings
- Manages API keys and service endpoints
- Environment variable loading

**Retrieval Service (`app/services/retrieval_service.py`)**
- Handles document retrieval from Qdrant
- Manages Cohere embedding generation
- Provides similarity search capabilities

### 2. Data Pipeline

**Sitemap Ingestion (`ingest_data_enhanced.py`)**
- Extracts URLs from sitemap.xml
- Scrapes content from web pages using trafilatura
- Chunks text into manageable segments
- Generates embeddings using Cohere
- Stores in Qdrant vector database

### 3. Frontend Integration (Docusaurus)

**Chat Interface Component (`frontend/src/components/ChatInterface.js`)**
- Floating chat widget with toggle functionality
- Real-time messaging with typing indicators
- Error handling and loading states
- Responsive design for all screen sizes

**Layout Integration (`frontend/src/theme/Layout/index.js`)**
- Integrated into all Docusaurus pages
- Maintains existing site functionality
- Automatically available on all textbook content

## Technology Stack

### Backend
- **FastAPI**: Modern, fast web framework for building APIs
- **Google Generative AI**: For language model responses (Gemini)
- **Cohere**: For text embedding generation
- **Qdrant**: Vector database for similarity search
- **Pydantic**: Data validation and settings management
- **Uvicorn**: ASGI server for running the application

### Frontend
- **React**: Component-based UI library
- **Docusaurus**: Static site generator for documentation
- **BrowserOnly**: Ensures client-side rendering only

## API Endpoints

### Query Endpoint
```
POST /query
```
**Request Body:**
```json
{
  "query": "Your question here",
  "top_k": 5
}
```

**Response:**
```json
{
  "query": "Your question here",
  "answer": "Generated answer from Gemini based on retrieved context",
  "sources": ["retrieved text chunk 1", "retrieved text chunk 2", ...]
}
```

### Health Check Endpoints
```
GET /health          - Overall system health
GET /health/qdrant   - Qdrant connectivity
GET /health/gemini   - Google Gemini connectivity
```

## Environment Variables

Required configuration in `.env`:

```env
GEMINI_API_KEY=your_google_gemini_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DOCUMENT_COLLECTION_NAME=humanoid_ai_book
EMBEDDING_MODEL=embed-english-v3.0
GEMINI_MODEL=gemini-2.5-flash
SITEMAP_URL=https://your-site.com/sitemap.xml
```

## Deployment

### Backend Deployment
1. Set up environment variables
2. Install dependencies: `pip install -r requirements.txt`
3. Run ingestion: `python ingest_data_enhanced.py`
4. Start API server: `python main.py` or `uvicorn app.api_main:app --host 0.0.0.0 --port 8000`

### Frontend Integration
1. The chat interface is automatically integrated into all Docusaurus pages
2. No additional configuration needed
3. The backend URL can be configured via environment variables

## Workflow

1. **Data Ingestion**: Content from the sitemap is scraped, chunked, embedded, and stored in Qdrant
2. **Query Processing**: User questions are received via the chat interface
3. **Retrieval**: Relevant documents are retrieved from Qdrant using vector similarity search
4. **Generation**: Google Gemini generates responses based on retrieved context
5. **Response**: Answer is returned to the user via the chat interface

## Security Considerations

- API keys are stored in environment variables
- Input validation through Pydantic models
- Rate limiting should be implemented in production
- CORS policies should be configured for production

## Performance Considerations

- Vector search in Qdrant provides fast similarity matching
- Cohere embeddings are efficient for retrieval
- Google Gemini provides quick response generation
- Caching can be implemented for frequently asked questions

## Troubleshooting

**Common Issues:**
- Ensure all API keys are properly configured
- Verify Qdrant connectivity and collection exists
- Check that the ingestion process completed successfully
- Confirm backend is running and accessible from frontend

**Testing:**
- Use `test_api.py` to verify API functionality
- Check health endpoints to verify service connectivity
- Validate that chat interface can communicate with backend