"""
Configuration module for environment variables and settings.
"""
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Configuration
    api_title: str = "AI-Native Textbook API"
    api_version: str = "1.0.0"
    environment: str = "development"

    # OpenAI Configuration
    openai_api_key: str = ""
    openai_embedding_model: str = "text-embedding-3-small"
    openai_chat_model: str = "gpt-4-turbo-preview"

    # Qdrant Configuration
    qdrant_url: str = ""
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "textbook_content"

    # Neon Postgres Configuration
    database_url: str = ""

    # Better-Auth Configuration
    better_auth_secret: str = ""
    better_auth_url: Optional[str] = None

    # Google Translate API Configuration
    google_translate_api_key: str = ""

    # Claude API Configuration (for Subagents)
    claude_api_key: str = ""

    # Application Settings
    chunk_size: int = 512
    chunk_overlap: int = 50
    embedding_dimensions: int = 1536
    rag_top_k: int = 20
    rag_final_k: int = 5

    class Config:
        env_file = ".env"
        case_sensitive = False


# Global settings instance
settings = Settings()
