"""
Configuration module for environment variables and settings.
"""
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API
    api_title: str = "AI-Native Textbook API"
    api_version: str = "1.0.0"
    environment: str = "development"

    # OpenAI
    openai_api_key: str = ""
    openai_embedding_model: str = "text-embedding-3-small"
    openai_chat_model: str = "gpt-4-turbo-preview"

    # Qdrant
    qdrant_url: str = ""
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "textbook_content"

    # Database
    database_url: str = ""

    # Auth
    better_auth_secret: str = ""
    better_auth_url: Optional[str] = None

    # APIs
    google_translate_api_key: str = ""
    claude_api_key: str = ""

    # RAG
    chunk_size: int = 512
    chunk_overlap: int = 50
    embedding_dimensions: int = 1536
    rag_top_k: int = 20
    rag_final_k: int = 5

    # Context7
    context7_enabled: bool = True
    context7_max_results: int = 2
    context7_timeout_seconds: float = 3.0

    # ChatKit
    chatkit_enabled: bool = True
    chatkit_max_threads_per_user: int = 100
    chatkit_max_messages_per_thread: int = 1000
    chatkit_stream_chunk_size: int = 50
    chatkit_pagination_default_limit: int = 50

    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"   # 🔥 THIS LINE IS IMPORTANT


settings = Settings()