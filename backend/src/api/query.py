"""
Query API endpoints for asking questions and getting answers.
"""
import time
from fastapi import APIRouter, HTTPException
from src.models.query import QueryRequest, QueryResponse
from src.services.chat_engine import chat_engine
from src.utils.validators import validate_query_length, validate_chunk_ids, sanitize_query
from src.utils.logger import get_logger

logger = get_logger("query_api")

router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query_book(request: QueryRequest):
    """
    Ask a question about the book and get an AI-generated answer with citations.

    Args:
        request: QueryRequest with query_text and optional selected_chunk_ids

    Returns:
        QueryResponse with answer, citations, and metadata
    """
    try:
        start_time = time.time()

        # Validate query
        if not validate_query_length(request.query_text):
            raise HTTPException(
                status_code=400,
                detail="Query text must be between 1 and 4000 characters"
            )

        # Sanitize query
        query_text = sanitize_query(request.query_text)

        # Validate chunk IDs if provided
        if request.selected_chunk_ids and not validate_chunk_ids(request.selected_chunk_ids):
            raise HTTPException(
                status_code=400,
                detail="Invalid chunk IDs provided or empty list"
            )

        logger.info(f"Processing query: '{query_text[:100]}...'")

        # Process query with chat engine
        result = await chat_engine.query(
            query_text=query_text,
            selected_chunk_ids=request.selected_chunk_ids
        )

        # Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)

        return QueryResponse(
            answer_text=result["answer_text"],
            citations=result["citations"],
            processing_time_ms=processing_time_ms,
            mode=result["mode"]
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/doc/{chunk_id}")
async def get_document_chunk(chunk_id: str):
    """
    Retrieve a specific chunk's full text and metadata by ID.

    Args:
        chunk_id: UUID of the chunk

    Returns:
        Chunk data with full text and metadata
    """
    try:
        from src.services.postgres_client import postgres_service

        chunk = await postgres_service.get_chunk(chunk_id)

        if not chunk:
            raise HTTPException(status_code=404, detail="Chunk not found")

        return {
            "chunk_id": str(chunk["id"]),
            "text": chunk["text"],
            "metadata": {
                "chapter_num": chunk.get("chapter_num"),
                "chapter_title": chunk.get("chapter_title"),
                "page_num": chunk.get("page_num"),
                "section_heading": chunk.get("section_heading"),
                "token_count": chunk["token_count"]
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching chunk: {e}")
        raise HTTPException(status_code=500, detail=str(e))
