"""
Chunk model for storing text segments.
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class Chunk(BaseModel):
    """Chunk entity model."""
    id: Optional[UUID] = None
    book_id: UUID
    chunk_index: int
    text: str
    chapter_num: Optional[int] = None
    chapter_title: Optional[str] = None
    page_num: Optional[int] = None
    section_heading: Optional[str] = None
    paragraph_index: Optional[int] = None
    token_count: int
    created_at: Optional[datetime] = None
