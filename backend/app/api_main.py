from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import google.generativeai as genai
from .config import settings
from .services.retrieval_service import RetrievalService
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0"
)

# Add CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
retrieval_service = RetrievalService()

# Initialize Google Gemini
genai.configure(api_key=settings.gemini_api_key)
gemini_model = genai.GenerativeModel(settings.gemini_model)

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5

class QueryResponse(BaseModel):
    query: str
    answer: str
    sources: List[str]

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.post("/query", response_model=QueryResponse)
def query_endpoint(request: QueryRequest):
    """Query the RAG system to get answers based on textbook content"""
    try:
        # Retrieve relevant documents
        sources_with_urls = retrieval_service.retrieve(request.query, request.top_k)
        sources = [source[0] for source in sources_with_urls]  # Extract just the text content

        if not sources:
            return QueryResponse(
                query=request.query,
                answer="I couldn't find any relevant information in the textbook to answer your question.",
                sources=[]
            )

        # Create context from retrieved documents
        context = "\n\n".join([f"Source: {source[:500]}..." for source in sources[:3]])  # Use top 3 sources, limit length

        # Create the prompt for Gemini
        prompt = f"""
        Based on the following context from the Physical AI & Humanoid Robotics textbook, please answer the question.
        If the context doesn't contain the information needed to answer the question, please say so.

        Context:
        {context}

        Question: {request.query}

        Please provide a comprehensive answer based on the textbook content.
        """

        # Get response from Google Gemini
        response = gemini_model.generate_content(
            prompt,
            generation_config={
                "temperature": 0.3,
            }
        )

        answer = response.text if response.text else "I couldn't generate a response based on the provided context."

        return QueryResponse(
            query=request.query,
            answer=answer,
            sources=sources
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/health")
def health_check():
    """Check if the API is running"""
    return {"status": "healthy", "service": "RAG Chatbot API"}

@app.get("/health/qdrant")
def qdrant_health():
    """Check Qdrant connectivity"""
    try:
        # Try to list collections to verify connectivity
        collections = retrieval_service.qdrant_client.get_collections()
        return {"status": "healthy", "service": "Qdrant"}
    except Exception as e:
        return {"status": "unhealthy", "service": "Qdrant", "error": str(e)}

@app.get("/health/gemini")
def gemini_health():
    """Check Google Gemini connectivity"""
    try:
        # Try a simple generation to check connectivity
        test_response = gemini_model.generate_content("Hello", generation_config={"temperature": 0.0})
        return {"status": "healthy", "service": "Google Gemini"}
    except Exception as e:
        return {"status": "unhealthy", "service": "Google Gemini", "error": str(e)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)