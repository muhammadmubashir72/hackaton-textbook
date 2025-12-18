from typing import List, Tuple
import cohere
from qdrant_client import QdrantClient
from ..config import settings

class RetrievalService:
    def __init__(self):
        self.cohere_client = cohere.Client(settings.cohere_api_key)
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.collection_name

    def get_embedding(self, text: str, input_type: str = "search_query") -> List[float]:
        """Get embedding vector from Cohere Embed v3"""
        response = self.cohere_client.embed(
            model=settings.embedding_model,
            input_type=input_type,
            texts=[text],
        )
        return response.embeddings[0]  # Return the first embedding

    def retrieve(self, query: str, top_k: int = 5) -> List[Tuple[str, str]]:
        """Retrieve relevant documents from Qdrant"""
        embedding = self.get_embedding(query)
        result = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=embedding,
            limit=top_k
        )
        # The query_points method returns a Response object with points attribute
        return [(point.payload["text"], point.payload.get("url", "")) for point in result.points]

    def retrieve_with_scores(self, query: str, top_k: int = 5) -> List[Tuple[str, str, float]]:
        """Retrieve relevant documents from Qdrant with scores"""
        embedding = self.get_embedding(query)
        result = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=embedding,
            limit=top_k
        )
        # The query_points method returns a Response object with points attribute
        return [(point.payload["text"], point.payload.get("url", ""), point.score) for point in result.points]