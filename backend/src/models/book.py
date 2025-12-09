"""
Book model for storing textbook metadata.
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class Book(BaseModel):
    """Book entity model."""
    id: Optional[UUID] = None
    title: str
    author: Optional[str] = None
    version: Optional[str] = None
    total_pages: Optional[int] = None
    ingestion_date: Optional[datetime] = None
    status: str = "pending"  # pending, processing, completed, failed
    created_at: Optional[datetime] = None
