import cohere
from qdrant_client import QdrantClient

# Initialize Cohere client
cohere_client = cohere.Client("ZbplmLDRTCbyRy2aWMyjz7lUJhbeWAClnubvHetU")
# Connect to Qdrant
qdrant = QdrantClient(
    url="https://f027e783-345a-4158-94c0-1dd4cb9d2946.us-east4-0.gcp.cloud.qdrant.io:6333", 
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Z2vu8iPFSk0cFa1oOWHzRvRpv4kqhn_NPgrf24c6jqs",
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding

def retrieve(query):
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="humanoid_ai_book_two",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]

# Test
print(retrieve("What data do you have?"))