from typing import List, Tuple
import cohere
from qdrant_client import QdrantClient
from ..config import settings
import google.generativeai as genai

class RetrievalService:
    def __init__(self):
        # Force use Gemini embeddings due to Cohere API quota limits
        self.use_gemini = True
        genai.configure(api_key=settings.gemini_api_key)
        print("RetrievalService initialized with Gemini embeddings")

        # Keep Cohere client as backup but don't use by default
        try:
            self.cohere_client = cohere.Client(settings.cohere_api_key)
        except Exception as e:
            print(f"Cohere client not available: {e}")
            self.cohere_client = None

        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.collection_name

    def get_embedding(self, text: str, input_type: str = "search_query") -> List[float]:
        """Get embedding vector from Cohere Embed v3 or Gemini as fallback"""
        try:
            if not self.use_gemini:
                response = self.cohere_client.embed(
                    model=settings.embedding_model,
                    input_type=input_type,
                    texts=[text],
                )
                return response.embeddings[0]  # Return the first embedding
        except Exception as e:
            print(f"Cohere embedding failed: {e}. Switching to Gemini.")
            self.use_gemini = True

        # Fallback to Gemini embeddings
        if self.use_gemini:
            genai.configure(api_key=settings.gemini_api_key)
            result = genai.embed_content(
                model="models/text-embedding-004",
                content=text,
                task_type="retrieval_query" if input_type == "search_query" else "retrieval_document"
            )
            return result['embedding']

    def retrieve(self, query: str, top_k: int = 5) -> List[Tuple[str, str]]:
        """Retrieve relevant documents from Qdrant"""
        embedding = self.get_embedding(query)
        result = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=embedding,
            limit=top_k
        )
        # The query_points method returns a Response object with points attribute
        # Try both 'content' and 'text' field names for backward compatibility
        return [(point.payload.get("content") or point.payload.get("text", ""), point.payload.get("url", "")) for point in result.points]

    def retrieve_with_scores(self, query: str, top_k: int = 5) -> List[Tuple[str, str, float]]:
        """Retrieve relevant documents from Qdrant with scores"""
        embedding = self.get_embedding(query)
        result = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=embedding,
            limit=top_k
        )
        # The query_points method returns a Response object with points attribute
        # Try both 'content' and 'text' field names for backward compatibility
        return [(point.payload.get("content") or point.payload.get("text", ""), point.payload.get("url", ""), point.score) for point in result.points]

    def check_translation_cache(self, original_text: str, target_language: str = "ur") -> str:
        """
        Check Qdrant metadata for cached translation of the given text
        """
        try:
            # Create a filter to search for the original text in metadata
            from qdrant_client.http import models
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="original_text.keyword",  # Use keyword for exact match
                        match=models.MatchValue(value=original_text)
                    ),
                    models.FieldCondition(
                        key="target_language",
                        match=models.MatchValue(value=target_language)
                    )
                ]
            )

            # Search in the same collection for cached translations
            result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query_filter=search_filter,
                limit=1
            )

            if result.points:
                # Found cached translation
                return result.points[0].payload.get("translated_text", None)

            return None
        except Exception as e:
            print(f"Error checking translation cache: {str(e)}")
            # If the index-based search fails, try a different approach using scroll
            try:
                # Use scroll to find matching records without requiring an index
                scroll_result = self.qdrant_client.scroll(
                    collection_name=self.collection_name,
                    scroll_filter=models.Filter(
                        must=[
                            models.FieldCondition(
                                key="target_language",
                                match=models.MatchValue(value=target_language)
                            )
                        ]
                    ),
                    limit=100  # Limit to avoid too many results
                )

                # Manually check each result for the original text
                for point in scroll_result.points:
                    if point.payload.get("original_text") == original_text:
                        return point.payload.get("translated_text", None)

                return None
            except Exception as e2:
                print(f"Error with fallback translation cache search: {str(e2)}")
                return None

    def store_translation_cache(self, original_text: str, translated_text: str, target_language: str = "ur"):
        """
        Store translation in Qdrant metadata for future retrieval
        """
        try:
            import uuid
            from qdrant_client.http.models import PointStruct

            # Create embedding for the original text to store with the translation
            embedding = self.get_embedding(original_text, input_type="search_document")

            # Create a new point with translation metadata
            # Ensure original_text is stored as a keyword field for exact matching
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "original_text": original_text,
                    "translated_text": translated_text,
                    "target_language": target_language,
                    "created_at": str(__import__('datetime').datetime.now())
                }
            )

            # Store in the same collection
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )
        except Exception as e:
            print(f"Error storing translation cache: {str(e)}")