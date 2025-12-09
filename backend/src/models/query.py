"""
Query model for user questions.
"""
from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
from uuid import UUID


class QueryRequest(BaseModel):
    """Request model for asking a question."""
    query_text: str
    selected_chunk_ids: Optional[List[str]] = None
    session_id: Optional[str] = None


class Citation(BaseModel):
    """Citation reference to a source chunk."""
    chunk_id: str
    excerpt: str
    chapter_page_ref: Optional[str] = None
    relevance_score: float


class QueryResponse(BaseModel):
    """Response model for query results."""
    answer_text: str
    citations: List[Citation]
    confidence_score: Optional[float] = None
    processing_time_ms: Optional[int] = None
    mode: str = "full-book"  # or "selected-text"
