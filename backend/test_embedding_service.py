"""
Test script to verify embedding service is working with Cohere or Gemini fallback
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add parent directory to path to import app modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app.services.retrieval_service import RetrievalService
from app.config import settings

def test_embedding_service():
    """Test if embedding service works"""
    try:
        print("=" * 60)
        print("Testing Embedding Service")
        print("=" * 60)

        # Initialize service
        print("\n1. Initializing RetrievalService...")
        service = RetrievalService()
        print(f"   ✓ Service initialized")
        print(f"   Using Gemini: {service.use_gemini}")

        # Test embedding generation
        print("\n2. Testing embedding generation...")
        test_query = "What is a humanoid robot?"
        embedding = service.get_embedding(test_query)
        print(f"   ✓ Generated embedding for: '{test_query}'")
        print(f"   Embedding dimension: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")

        # Test retrieval
        print("\n3. Testing document retrieval...")
        results = service.retrieve(test_query, top_k=3)
        print(f"   ✓ Retrieved {len(results)} documents")

        if results:
            print("\n   Top Result:")
            content, url = results[0]
            print(f"   URL: {url}")
            print(f"   Content preview: {content[:200]}...")

        print("\n" + "=" * 60)
        print("✓ All tests passed successfully!")
        print("=" * 60)

        return True

    except Exception as e:
        print("\n" + "=" * 60)
        print(f"✗ Test failed with error:")
        print(f"  {str(e)}")
        print("=" * 60)
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_embedding_service()
    sys.exit(0 if success else 1)
