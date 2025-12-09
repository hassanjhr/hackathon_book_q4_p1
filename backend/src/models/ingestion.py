"""
Ingestion request and response models.
"""
from pydantic import BaseModel
from typing import Optional
from uuid import UUID


class IngestionRequest(BaseModel):
    """Request model for book ingestion."""
    title: str
    author: Optional[str] = None
    version: Optional[str] = None


class IngestionResponse(BaseModel):
    """Response model for ingestion job."""
    job_id: UUID
    status: str
    total_chunks: Optional[int] = None
    message: str
