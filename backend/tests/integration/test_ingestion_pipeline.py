"""
Integration tests for the ingestion pipeline.

Tests the complete flow:
1. Upload book content
2. Chunking with tiktoken
3. Embedding generation with OpenAI
4. Storage in Qdrant (vectors) and Neon (metadata)
5. Verification of data integrity
"""
import pytest
import asyncio
from pathlib import Path
from uuid import uuid4


@pytest.mark.integration
@pytest.mark.asyncio
async def test_ingestion_pipeline_end_to_end():
    """
    Test the complete ingestion pipeline from file upload to storage.

    Prerequisites:
    - Qdrant service running and accessible
    - Neon Postgres running and accessible
    - OpenAI API key configured
    """
    pytest.skip("Integration test - requires actual services (Qdrant, Neon, OpenAI)")

    from src.services.ingestion_service import ingestion_service
    from src.services.qdrant_client import qdrant_service
    from src.services.postgres_client import postgres_service

    # Read test fixture
    test_file = Path(__file__).parent.parent / "fixtures" / "sample_book.txt"
    with open(test_file, 'r') as f:
        text_content = f.read()

    # Create unique book ID for this test
    book_id = str(uuid4())

    try:
        # Step 1: Ingest the book
        result = await ingestion_service.ingest_text(
            text=text_content,
            title="Test Robotics Book",
            author="Test Author",
            version="1.0"
        )

        # Verify ingestion completed successfully
        assert result["status"] == "completed"
        assert result["total_chunks"] > 0
        chunk_count = result["total_chunks"]

        # Step 2: Verify chunks are in Postgres
        async with postgres_service.pool.acquire() as conn:
            pg_chunks = await conn.fetch(
                "SELECT COUNT(*) as count FROM chunks WHERE book_id = $1",
                book_id
            )
            assert pg_chunks[0]['count'] == chunk_count

        # Step 3: Verify vectors are in Qdrant
        # Search with a test query to verify vectors exist
        from src.services.embedder import embedder_service
        test_query = "What are the main components of a humanoid robot?"
        query_vector = await embedder_service.embed_text(test_query)

        search_results = qdrant_service.search_vectors(
            query_vector=query_vector,
            limit=5
        )

        assert len(search_results) > 0
        assert all('chunk_id' in result for result in search_results)

        # Step 4: Verify chunk integrity (text content preserved)
        async with postgres_service.pool.acquire() as conn:
            chunk = await conn.fetchrow(
                "SELECT text, token_count FROM chunks WHERE book_id = $1 LIMIT 1",
                book_id
            )
            assert len(chunk['text']) > 0
            assert chunk['token_count'] <= 512  # Max chunk size
            assert chunk['token_count'] > 0

        print(f"✓ Integration test passed: {chunk_count} chunks ingested successfully")

    finally:
        # Cleanup: Remove test data
        async with postgres_service.pool.acquire() as conn:
            await conn.execute("DELETE FROM chunks WHERE book_id = $1", book_id)
            await conn.execute("DELETE FROM books WHERE id = $1", book_id)

        # Note: Qdrant cleanup would require deleting by book_id filter
        # This is left as manual cleanup in test environment


@pytest.mark.integration
@pytest.mark.asyncio
async def test_chunking_respects_token_limits():
    """Test that chunker creates chunks within token limits."""
    pytest.skip("Integration test - requires tiktoken")

    from src.services.chunker import chunker_service

    # Create a long text that will require multiple chunks
    long_text = "This is a test sentence. " * 1000

    chunks = chunker_service.chunk_text(long_text)

    # Verify all chunks are within limits
    assert all(chunk['token_count'] <= 512 for chunk in chunks)
    assert all(chunk['token_count'] > 0 for chunk in chunks)

    # Verify we have multiple chunks
    assert len(chunks) > 1

    # Verify chunk indices are sequential
    indices = [chunk['chunk_index'] for chunk in chunks]
    assert indices == list(range(len(chunks)))

    print(f"✓ Created {len(chunks)} chunks, all within token limits")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_embedding_generation():
    """Test OpenAI embedding generation."""
    pytest.skip("Integration test - requires OpenAI API")

    from src.services.embedder import embedder_service

    test_texts = [
        "Humanoid robots require advanced sensors.",
        "Actuators enable robot movement.",
        "Control systems coordinate behavior."
    ]

    # Test single embedding
    vector1 = await embedder_service.embed_text(test_texts[0])
    assert len(vector1) == 1536  # text-embedding-3-small dimensions
    assert all(isinstance(v, float) for v in vector1)

    # Test batch embedding
    vectors = await embedder_service.embed_batch(test_texts)
    assert len(vectors) == len(test_texts)
    assert all(len(v) == 1536 for v in vectors)

    print(f"✓ Generated embeddings: {len(vectors)} vectors of dimension 1536")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_qdrant_vector_storage():
    """Test vector storage and retrieval from Qdrant."""
    pytest.skip("Integration test - requires Qdrant service")

    from src.services.qdrant_client import qdrant_service
    from src.services.embedder import embedder_service

    # Generate test embedding
    test_text = "Test robotics content for vector storage"
    vector = await embedder_service.embed_text(test_text)

    test_chunk_id = str(uuid4())

    try:
        # Insert vector
        qdrant_service.insert_vectors(
            vectors=[vector],
            chunk_ids=[test_chunk_id],
            metadata=[{
                "book_id": "test-book",
                "chapter_num": 1,
                "text": test_text
            }]
        )

        # Search for the vector
        results = qdrant_service.search_vectors(
            query_vector=vector,
            limit=1
        )

        assert len(results) > 0
        assert results[0]['chunk_id'] == test_chunk_id
        assert results[0]['score'] > 0.99  # Should be nearly identical

        print(f"✓ Vector storage and retrieval successful, score: {results[0]['score']:.4f}")

    finally:
        # Cleanup would require Qdrant delete by ID
        # Left for manual cleanup in test environment
        pass


@pytest.mark.integration
@pytest.mark.asyncio
async def test_postgres_chunk_storage():
    """Test chunk metadata storage in Postgres."""
    pytest.skip("Integration test - requires Postgres connection")

    from src.services.postgres_client import postgres_service

    book_id = str(uuid4())
    chunk_id = str(uuid4())

    try:
        # Insert test book
        await postgres_service.insert_book({
            "id": book_id,
            "title": "Test Book",
            "author": "Test Author",
            "version": "1.0",
            "status": "processing"
        })

        # Insert test chunk
        chunk_data = {
            "id": chunk_id,
            "book_id": book_id,
            "chunk_index": 0,
            "text": "Test chunk content",
            "chapter_num": 1,
            "chapter_title": "Introduction",
            "page_num": 1,
            "token_count": 10
        }

        await postgres_service.insert_chunks([chunk_data])

        # Retrieve and verify
        retrieved_chunk = await postgres_service.get_chunk(chunk_id)

        assert retrieved_chunk is not None
        assert retrieved_chunk['id'] == chunk_id
        assert retrieved_chunk['book_id'] == book_id
        assert retrieved_chunk['text'] == chunk_data['text']
        assert retrieved_chunk['token_count'] == chunk_data['token_count']

        print(f"✓ Chunk metadata stored and retrieved successfully")

    finally:
        # Cleanup
        async with postgres_service.pool.acquire() as conn:
            await conn.execute("DELETE FROM chunks WHERE id = $1", chunk_id)
            await conn.execute("DELETE FROM books WHERE id = $1", book_id)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_ingestion_error_handling():
    """Test error handling during ingestion."""
    pytest.skip("Integration test - requires services")

    from src.services.ingestion_service import ingestion_service

    # Test with empty text
    with pytest.raises(ValueError):
        await ingestion_service.ingest_text(
            text="",
            title="Empty Book",
            author="Test"
        )

    # Test with invalid parameters
    with pytest.raises(ValueError):
        await ingestion_service.ingest_text(
            text=None,
            title="Invalid Book",
            author="Test"
        )

    print("✓ Error handling tests passed")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "integration"])
