# Physical AI & Humanoid Robotics RAG System for Hugging Face Spaces

This project implements a Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics textbook, optimized for deployment on Hugging Face Spaces.

## Deployment to Hugging Face Spaces

### Prerequisites

1. A Hugging Face account
2. Access to Hugging Face Spaces
3. Valid API keys for:
   - Google Gemini
   - Cohere
   - Qdrant (vector database)

### Steps to Deploy

1. **Fork or import this repository** to your Hugging Face account

2. **Configure the Space**:
   - Choose "Docker" as the SDK
   - Use the provided Dockerfile in this repository

3. **Set environment variables** in your Space settings (Settings → Secrets):
   - `GEMINI_API_KEY`: Your Google Gemini API key
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant cloud URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - Optionally: `SITEMAP_URL`, `LOCAL_DOCS_DIR`, `COLLECTION_NAME`

4. **Prepare your vector database**:
   - Before your Space will work properly, you need to populate your Qdrant collection with the textbook content
   - Run the ingestion script locally or in a separate environment:
     ```bash
     python ingest_data_enhanced.py
     ```

5. **The Space will automatically deploy** using the Dockerfile configuration

## API Endpoints Available

Once deployed, your API will be available at:
- `https://YOUR_USERNAME-space-name.hf.space/` - API root
- `https://YOUR_USERNAME-space-name.hf.space/query` - Query endpoint
- `https://YOUR_USERNAME-space-name.hf.space/health` - Health check
- `https://YOUR_USERNAME-space-name.hf.space/docs` - Interactive API documentation

### Query Endpoint Usage

Send a POST request to `/query` with the following JSON format:
```json
{
  "query": "Your question about the textbook",
  "top_k": 5
}
```

The API will return a response with the answer and source documents.

## Architecture

1. **Data Ingestion**: Content from web sources and local documents is embedded using Cohere and stored in Qdrant
2. **Backend API**: FastAPI service that handles queries and generates responses using Google Gemini
3. **Vector Search**: Qdrant database provides semantic search capabilities for RAG

## Important Notes

- The initial data ingestion is required before the system can answer questions
- Make sure your Qdrant collection contains the embedded textbook content
- API keys are securely stored as Space secrets and not included in the repository
- The Space automatically scales based on usage