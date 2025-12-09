"""
Smoke tests for RAG chatbot MVP.

These tests verify basic functionality:
1. Ingestion: Upload book → verify chunks in Qdrant
2. Query: Ask question → verify answer with citations
"""
import pytest
import asyncio
from pathlib import Path


# Note: These are integration tests that require actual services running
# For local testing, ensure Qdrant and Neon are accessible with valid credentials

@pytest.mark.asyncio
async def test_health_check():
    """Test that the API health check endpoint works."""
    from httpx import AsyncClient
    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"


@pytest.mark.asyncio
async def test_ingestion_pipeline():
    """
    Test the complete ingestion pipeline.

    1. Upload sample book
    2. Verify chunks are created
    3. Check Qdrant has vectors
    """
    # This test requires actual database connections
    # Skip if not in integration test environment

    pytest.skip("Integration test - requires actual services")

    # from src.services.ingestion_service import ingestion_service
    #
    # # Read sample book
    # sample_file = Path(__file__).parent / "fixtures" / "sample_book.txt"
    # with open(sample_file, 'r') as f:
    #     text = f.read()
    #
    # # Ingest
    # result = await ingestion_service.ingest_text(
    #     text=text,
    #     title="Test Book",
    #     author="Test Author"
    # )
    #
    # assert result["status"] == "completed"
    # assert result["total_chunks"] > 0


@pytest.mark.asyncio
async def test_query_pipeline():
    """
    Test the query pipeline.

    1. Ask a question
    2. Verify answer is returned
    3. Check citations are present
    """
    # This test requires actual database connections and ingested content
    # Skip if not in integration test environment

    pytest.skip("Integration test - requires actual services and ingested content")

    # from src.services.chat_engine import chat_engine
    #
    # result = await chat_engine.query(
    #     query_text="What are the main components of a humanoid robot?"
    # )
    #
    # assert "answer_text" in result
    # assert len(result["answer_text"]) > 0
    # assert len(result["citations"]) > 0
    # assert result["mode"] == "full-book"


def test_chunker_service():
    """Test that chunker creates chunks with proper token counts."""
    from src.services.chunker import chunker_service

    text = "This is a test sentence. " * 100  # Create text that will need multiple chunks

    chunks = chunker_service.chunk_text(text)

    assert len(chunks) > 0
    assert all(chunk["token_count"] <= 512 for chunk in chunks)
    assert all("text" in chunk for chunk in chunks)


def test_validators():
    """Test validation functions."""
    from src.utils.validators import (
        validate_query_length,
        validate_chunk_id,
        validate_chunk_ids,
        sanitize_query
    )

    # Query length
    assert validate_query_length("Valid query")
    assert not validate_query_length("")
    assert not validate_query_length("x" * 5000)

    # Chunk ID (UUID format)
    assert validate_chunk_id("123e4567-e89b-12d3-a456-426614174000")
    assert not validate_chunk_id("invalid-id")

    # Chunk IDs list
    assert validate_chunk_ids(["123e4567-e89b-12d3-a456-426614174000"])
    assert not validate_chunk_ids([])
    assert not validate_chunk_ids(["invalid"])

    # Sanitize
    assert sanitize_query("  test  query  ") == "test query"
    assert "\x00" not in sanitize_query("test\x00query")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
