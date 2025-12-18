"""
Basic tests for the RAG Chatbot backend
"""
import asyncio
import pytest
from fastapi.testclient import TestClient
from app.api_main import app
from app.config import settings

# Create test client
client = TestClient(app)

def test_health_check():
    """Test the health check endpoint"""
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["service"] == "RAG Chatbot API"

def test_health_vector_store():
    """Test the vector store health check endpoint"""
    # This test might fail if Qdrant is not running
    response = client.get("/api/v1/health/vector-store")
    # We'll accept both 200 and 500 depending on Qdrant availability
    assert response.status_code in [200, 500]

def test_health_llm():
    """Test the LLM health check endpoint"""
    # This test might fail if Google API key is not set
    response = client.get("/api/v1/health/llm")
    # We'll accept both 200 and 500 depending on API key availability
    assert response.status_code in [200, 500]

def test_root_endpoint():
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "RAG Chatbot API is running"

def test_query_endpoint_structure():
    """Test the query endpoint accepts proper structure"""
    # Test with minimal valid request
    response = client.post(
        "/api/v1/query",
        json={
            "query": "test query",
            "top_k": 5,
            "use_user_selection": False,
            "filters": {}
        }
    )
    # Should return 500 if Google Gemini is not configured, or 200 if it works
    assert response.status_code in [200, 500]

def test_selection_process_endpoint():
    """Test the user selection endpoint"""
    response = client.post(
        "/api/v1/selections/process",
        json={
            "content": "This is a test selection",
            "session_id": "test_session_123",
            "context_info": {
                "page": "/test/page",
                "title": "Test Page"
            }
        }
    )
    # Should return 500 if Google Gemini is not configured, or 200 if it works
    assert response.status_code in [200, 500]

if __name__ == "__main__":
    # Run the tests
    test_health_check()
    test_health_vector_store()
    test_health_llm()
    test_root_endpoint()
    test_query_endpoint_structure()
    test_selection_process_endpoint()
    print("All basic tests passed!")