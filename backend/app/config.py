import os
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Gemini
    gemini_api_key: str
    gemini_model: str = "gemini-1.5-flash"

    # Cohere
    cohere_api_key: str
    embedding_model: str = "embed-english-v3.0"

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    collection_name: str = "humanoid_ai_book"

    # JWT
    jwt_secret: str = os.getenv("JWT_SECRET", "your-jwt-secret-change-in-production")
    jwt_secret_key: str = os.getenv("JWT_SECRET_KEY", "your-default-secret-key-change-in-production")
    jwt_algorithm: str = os.getenv("JWT_ALGORITHM", "HS256")
    jwt_access_token_expiry: int = int(os.getenv("JWT_ACCESS_TOKEN_EXPIRY", "900"))  # in seconds
    jwt_access_token_expiry_minutes: int = int(os.getenv("JWT_ACCESS_TOKEN_EXPIRY_MINUTES", "15"))
    jwt_refresh_token_expiry: int = int(os.getenv("JWT_REFRESH_TOKEN_EXPIRY", "604800"))  # in seconds
    jwt_refresh_token_expiry_days: int = int(os.getenv("JWT_REFRESH_TOKEN_EXPIRY_DAYS", "7"))
    jwt_issuer: str = os.getenv("JWT_ISSUER", "physical-ai-textbook")

    # Database
    database_url: str = os.getenv("DATABASE_URL", "sqlite:///./textbook_auth.db")
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")
    neon_api_key: str = os.getenv("NEON_API_KEY", "")

    # Connection Pool Settings
    db_pool_min: int = int(os.getenv("DB_POOL_MIN", "2"))
    db_pool_max: int = int(os.getenv("DB_POOL_MAX", "10"))
    db_statement_timeout: int = int(os.getenv("DB_STATEMENT_TIMEOUT", "30000"))
    db_idle_timeout: int = int(os.getenv("DB_IDLE_TIMEOUT", "30000"))
    db_retry_attempts: int = int(os.getenv("DB_RETRY_ATTEMPTS", "3"))
    db_retry_delay: float = float(os.getenv("DB_RETRY_DELAY", "1.0"))

    # Redis
    redis_url: str = os.getenv("REDIS_URL", "redis://localhost:6379")
    redis_password: str = os.getenv("REDIS_PASSWORD", "")
    redis_db: int = int(os.getenv("REDIS_DB", "0"))
    redis_max_connections: int = int(os.getenv("REDIS_MAX_CONNECTIONS", "10"))
    redis_socket_timeout: int = int(os.getenv("REDIS_SOCKET_TIMEOUT", "5"))
    redis_socket_connect_timeout: int = int(os.getenv("REDIS_SOCKET_CONNECT_TIMEOUT", "5"))

    # Better Auth
    better_auth_secret: str = os.getenv("BETTER_AUTH_SECRET", "")
    better_auth_url: str = os.getenv("BETTER_AUTH_URL", "http://localhost:3000/api/auth")

    # Encryption
    encryption_key: str = os.getenv("ENCRYPTION_KEY", "")

    # CORS
    cors_allowed_origins: str = os.getenv("CORS_ALLOWED_ORIGINS", "http://localhost:3000,http://localhost:8001")
    cors_allow_credentials: str = os.getenv("CORS_ALLOW_CREDENTIALS", "true")

    # Rate Limiting
    rate_limit_enabled: str = os.getenv("RATE_LIMIT_ENABLED", "true")
    rate_limit_signup: int = int(os.getenv("RATE_LIMIT_SIGNUP", "5"))  # per hour
    rate_limit_signin: int = int(os.getenv("RATE_LIMIT_SIGNIN", "10"))  # per hour
    rate_limit_api: int = int(os.getenv("RATE_LIMIT_API", "100"))  # per minute

    # Logging
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    log_format: str = os.getenv("LOG_FORMAT", "json")
    log_file_path: str = os.getenv("LOG_FILE_PATH", "logs/app.log")

    # Google OAuth
    google_client_id: str = os.getenv("GOOGLE_CLIENT_ID", "")
    google_client_secret: str = os.getenv("GOOGLE_CLIENT_SECRET", "")
    google_redirect_uri: str = os.getenv("GOOGLE_REDIRECT_URI", f"http://localhost:{os.getenv('PORT', '8001')}/api/auth/oauth/google/callback")

    # Frontend and Backend URLs
    frontend_url: str = os.getenv("FRONTEND_URL", "http://localhost:3000")
    backend_url: str = os.getenv("BACKEND_URL", f"http://localhost:{os.getenv('PORT', '8001')}")

    # EXTRA fields (your .env includes these)
    sitemap_url: str
    local_docs_dir: str = "./specs"
    port: int = int(os.getenv("PORT", "8001"))
    host: str = os.getenv("HOST", "localhost")

    class Config:
        env_file = ".env"
        extra = "allow"   # allow extra env vars

settings = Settings()
