"""
Test script to verify Qdrant connection and check for existing data
"""
import cohere
from qdrant_client import QdrantClient
from app.config import settings

def test_connection():
    try:
        # Initialize clients
        cohere_client = cohere.Client(settings.cohere_api_key)
        qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        
        print("Testing Qdrant connection...")
        
        # Check if collection exists
        try:
            collection_info = qdrant_client.get_collection(settings.collection_name)
            print(f"Collection '{settings.collection_name}' exists")
            print(f"Points count: {collection_info.points_count}")
        except Exception as e:
            print(f"Collection '{settings.collection_name}' does not exist: {e}")
            return
        
        # Try to get some points to see if there's data
        try:
            points = qdrant_client.scroll(
                collection_name=settings.collection_name,
                limit=2  # Just get 2 points to check
            )
            
            if points[0]:  # If we got points
                print("Sample data exists in collection:")
                for idx, point in enumerate(points[0][:2]):  # Show first 2 points
                    print(f"  Point {idx + 1}:")
                    print(f"    Text preview: {point.payload['text'][:100]}...")
                    print(f"    Source: {point.payload.get('source', 'Unknown')}")
            else:
                print("No points found in the collection.")
                
        except Exception as e:
            print(f"Could not retrieve sample data: {e}")
            
        # Test embedding generation
        try:
            test_embedding = cohere_client.embed(
                model=settings.embedding_model,
                input_type="search_query",
                texts=["test query"],
            )
            print("✓ Cohere embedding generation works")
        except Exception as e:
            print(f"✗ Cohere embedding failed: {e}")
        
        # Test a simple query if there's data
        if collection_info.points_count > 0:
            try:
                # Get embedding for a test query
                query_embedding = cohere_client.embed(
                    model=settings.embedding_model,
                    input_type="search_query",
                    texts=["What is humanoid robotics?"],
                ).embeddings[0]
                
                # Search for similar points
                results = qdrant_client.query_points(
                    collection_name=settings.collection_name,
                    query=query_embedding,
                    limit=3
                )
                
                print(f"✓ Query search works, found {len(results.points)} results")
                
                if results.points:
                    print("Top result preview:")
                    print(f"  Text: {results.points[0].payload['text'][:200]}...")
                    
            except Exception as e:
                print(f"✗ Query search failed: {e}")
    
    except Exception as e:
        print(f"Connection test failed: {e}")

if __name__ == "__main__":
    test_connection()