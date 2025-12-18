# Quick Start Guide

Follow these steps to get the RAG Chatbot system up and running quickly.

## Prerequisites

- Python 3.8+
- Node.js (for Docusaurus frontend)
- Google API key with Gemini access
- Cohere API key
- Qdrant Cloud account

## Step 1: Clone and Setup Backend

```bash
# Clone the repository
git clone <your-repo-url>
cd MMNI/backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Step 2: Configure Environment Variables

```bash
# Copy the environment file
cp .env .env.local

# Edit .env.local with your API keys
nano .env.local
```

Add your API keys:
```env
GEMINI_API_KEY=your_google_gemini_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Step 3: Ingest Data into Vector Database

```bash
# Run the ingestion script to populate Qdrant with textbook content
python ingest_data_enhanced.py
```

This will:
- Extract URLs from the sitemap
- Scrape content from web pages
- Chunk text into manageable segments
- Generate embeddings using Cohere
- Store in Qdrant vector database

## Step 4: Start the Backend API

```bash
# Start the FastAPI server
python main.py
```

The API will be available at `http://localhost:8000`

## Step 5: Test the API

```bash
# Test the health endpoint
curl http://localhost:8000/health

# Test a query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "top_k": 5
  }'
```

## Step 6: Integrate with Docusaurus Frontend

The chat interface is automatically integrated into your Docusaurus site through the layout wrapper. If you need to rebuild your Docusaurus site:

```bash
cd frontend
npm install  # or yarn install
npm run build  # or yarn build
npm run serve  # to serve locally
```

## Step 7: Using the Chat Interface

1. Navigate to your Docusaurus site
2. Look for the floating chat button (💬) in the bottom-right corner
3. Click to open the chat interface
4. Type your questions about the textbook content
5. The AI assistant will retrieve relevant information and provide answers

## Common Issues and Solutions

### Issue: API Keys Not Working
**Solution:** Verify that your API keys are correct and have the necessary permissions.

### Issue: Qdrant Connection Error
**Solution:** Check that your Qdrant URL and API key are correct, and that the collection exists.

### Issue: No Results from Queries
**Solution:** Ensure the ingestion process completed successfully and data was stored in Qdrant.

### Issue: Slow Response Times
**Solution:** Check your API quotas and consider implementing caching for frequently asked questions.

## Development Mode

For development with auto-reload:

```bash
# Instead of python main.py, use:
uvicorn app.api_main:app --reload --host 0.0.0.0 --port 8000
```

## Environment Configuration

You can override the backend URL in the frontend by setting the `BACKEND_URL` environment variable:

```bash
# In your Docusaurus .env file
BACKEND_URL=https://your-deployed-backend.com
```

## Next Steps

- Customize the chat interface styling to match your site
- Add more sophisticated error handling
- Implement user feedback mechanisms
- Set up monitoring and logging
- Deploy to production