import os
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Gemini
    gemini_api_key: str
    gemini_model: str = "gemini-2.5-flash"

    # Cohere
    cohere_api_key: str
    embedding_model: str = "embed-english-v3.0"

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    collection_name: str = "humanoid_ai_book"

    # EXTRA fields (your .env includes these)
    sitemap_url: str
    local_docs_dir: str = "./specs"

    class Config:
        env_file = ".env"
        extra = "allow"   # allow extra env vars

settings = Settings()
