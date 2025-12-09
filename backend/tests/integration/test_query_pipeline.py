"""
Integration tests for the query/RAG pipeline.

Tests the complete flow:
1. Query submission
2. Semantic search in Qdrant
3. OpenAI Agents tool-calling
4. Answer generation with citations
5. Selected-text mode filtering
"""
import pytest
import asyncio
from pathlib import Path


@pytest.mark.integration
@pytest.mark.asyncio
async def test_query_pipeline_end_to_end():
    """
    Test the complete query pipeline from question to answer with citations.

    Prerequisites:
    - Book content already ingested
    - Qdrant and Neon services running
    - OpenAI API key configured
    """
    pytest.skip("Integration test - requires actual services and ingested content")

    from src.services.chat_engine import chat_engine

    # Test query about robotics
    query_text = "What are the main components of a humanoid robot?"

    result = await chat_engine.query(query_text=query_text)

    # Verify response structure
    assert "answer_text" in result
    assert "citations" in result
    assert "mode" in result

    # Verify answer quality
    assert len(result["answer_text"]) > 0
    assert result["mode"] == "full-book"

    # Verify citations
    assert len(result["citations"]) > 0
    for citation in result["citations"]:
        assert "chunk_id" in citation
        assert "excerpt" in citation
        assert "chapter_page_ref" in citation
        assert "relevance_score" in citation
        assert citation["relevance_score"] > 0

    # Verify answer relevance (should mention key components)
    answer_lower = result["answer_text"].lower()
    assert any(keyword in answer_lower for keyword in [
        "sensor", "actuator", "control", "component", "system"
    ])

    print(f"✓ Query pipeline test passed")
    print(f"  Answer: {result['answer_text'][:100]}...")
    print(f"  Citations: {len(result['citations'])}")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_selected_text_mode():
    """
    Test selected-text mode with chunk ID filtering.

    Verifies that when chunk IDs are provided, only those chunks are searched.
    """
    pytest.skip("Integration test - requires actual services and ingested content")

    from src.services.chat_engine import chat_engine
    from src.services.postgres_client import postgres_service

    # Get some actual chunk IDs from the database
    async with postgres_service.pool.acquire() as conn:
        chunks = await conn.fetch(
            "SELECT id FROM chunks LIMIT 3"
        )
        chunk_ids = [str(chunk['id']) for chunk in chunks]

    assert len(chunk_ids) > 0

    # Query with selected chunk IDs
    query_text = "Explain this section in simple terms"

    result = await chat_engine.query(
        query_text=query_text,
        selected_chunk_ids=chunk_ids
    )

    # Verify selected-text mode was used
    assert result["mode"] == "selected-text"

    # Verify citations only reference selected chunks
    cited_chunk_ids = [cit["chunk_id"] for cit in result["citations"]]
    assert all(cid in chunk_ids for cid in cited_chunk_ids)

    print(f"✓ Selected-text mode test passed")
    print(f"  Selected chunks: {len(chunk_ids)}")
    print(f"  Citations: {len(result['citations'])}")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_semantic_search_quality():
    """
    Test semantic search returns relevant results.

    Verifies that similar queries return similar chunks.
    """
    pytest.skip("Integration test - requires actual services")

    from src.services.embedder import embedder_service
    from src.services.qdrant_client import qdrant_service

    # Test queries with known semantic similarity
    query1 = "How do robots move?"
    query2 = "What enables robot locomotion?"

    # Embed both queries
    vector1 = await embedder_service.embed_text(query1)
    vector2 = await embedder_service.embed_text(query2)

    # Search with both
    results1 = qdrant_service.search_vectors(vector1, limit=5)
    results2 = qdrant_service.search_vectors(vector2, limit=5)

    # Results should overlap (semantically similar queries)
    chunk_ids1 = set(r['chunk_id'] for r in results1)
    chunk_ids2 = set(r['chunk_id'] for r in results2)

    overlap = len(chunk_ids1.intersection(chunk_ids2))

    # Expect at least some overlap
    assert overlap > 0, "Semantically similar queries should return overlapping results"

    print(f"✓ Semantic search quality test passed")
    print(f"  Overlap: {overlap}/{min(len(results1), len(results2))} chunks")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_openai_agents_tool_calling():
    """
    Test that OpenAI Agents correctly invoke tools.

    Verifies automatic tool-calling for search_chunks and get_metadata.
    """
    pytest.skip("Integration test - requires OpenAI API and ingested content")

    from src.services.chat_engine import chat_engine
    from src.tools.search_chunks import search_chunks
    from src.tools.get_metadata import get_metadata

    query_text = "What sensors are used in humanoid robots?"

    # The chat engine should automatically call search_chunks
    result = await chat_engine.query(query_text=query_text)

    # Verify tools were called (check that we have citations)
    assert len(result["citations"]) > 0

    # Verify we can call tools directly
    search_result = await search_chunks(query_text, limit=5)
    assert "chunks" in search_result
    assert len(search_result["chunks"]) > 0

    # Test metadata retrieval
    chunk_id = search_result["chunks"][0]["chunk_id"]
    metadata = await get_metadata(chunk_id)
    assert "text" in metadata
    assert "metadata" in metadata

    print(f"✓ Tool-calling test passed")
    print(f"  Search returned: {len(search_result['chunks'])} chunks")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_citation_accuracy():
    """
    Test that citations accurately reference source chunks.

    Verifies that cited chunks actually contain relevant content.
    """
    pytest.skip("Integration test - requires actual services")

    from src.services.chat_engine import chat_engine
    from src.services.postgres_client import postgres_service

    query_text = "What are actuators?"

    result = await chat_engine.query(query_text=query_text)

    # Verify each citation
    for citation in result["citations"]:
        chunk_id = citation["chunk_id"]

        # Retrieve full chunk
        chunk = await postgres_service.get_chunk(chunk_id)
        assert chunk is not None

        # Verify excerpt appears in chunk text
        excerpt = citation["excerpt"]
        assert excerpt in chunk["text"], \
            f"Citation excerpt not found in source chunk"

        # Verify chapter/page reference is formatted correctly
        assert citation["chapter_page_ref"], "Missing chapter/page reference"

    print(f"✓ Citation accuracy test passed")
    print(f"  Verified {len(result['citations'])} citations")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_query_performance():
    """
    Test that queries complete within performance requirements.

    Target: < 5 seconds for 95% of queries (SC-001)
    """
    pytest.skip("Integration test - requires actual services")

    import time
    from src.services.chat_engine import chat_engine

    test_queries = [
        "What are the main components of a humanoid robot?",
        "How do sensors work?",
        "Explain actuation systems",
        "What is a control system?",
        "How do robots perceive their environment?"
    ]

    response_times = []

    for query in test_queries:
        start_time = time.time()

        result = await chat_engine.query(query_text=query)

        elapsed = time.time() - start_time
        response_times.append(elapsed)

        assert result is not None

    # Calculate statistics
    avg_time = sum(response_times) / len(response_times)
    max_time = max(response_times)

    # Performance assertions
    assert max_time < 5.0, f"Query took {max_time:.2f}s, exceeds 5s limit"
    assert avg_time < 3.0, f"Average time {avg_time:.2f}s is too high"

    print(f"✓ Performance test passed")
    print(f"  Average: {avg_time:.2f}s")
    print(f"  Max: {max_time:.2f}s")
    print(f"  Min: {min(response_times):.2f}s")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_hallucination_prevention():
    """
    Test that the system doesn't hallucinate when info is not in the book.

    Verifies FR-011: System states "information not available" for out-of-scope questions.
    """
    pytest.skip("Integration test - requires actual services")

    from src.services.chat_engine import chat_engine

    # Query about something definitely not in a robotics textbook
    out_of_scope_query = "What is the recipe for chocolate cake?"

    result = await chat_engine.query(query_text=out_of_scope_query)

    answer_lower = result["answer_text"].lower()

    # Should indicate information not available
    assert any(phrase in answer_lower for phrase in [
        "not available",
        "not found",
        "cannot find",
        "not in the book",
        "no information"
    ]), "System should state when information is not available"

    print(f"✓ Hallucination prevention test passed")
    print(f"  Response: {result['answer_text'][:100]}...")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_multi_turn_conversation():
    """
    Test conversation context handling across multiple turns.

    Note: Full conversation history tracking is deferred (Phase 2.6).
    This test verifies basic multi-turn capability.
    """
    pytest.skip("Integration test - conversation history not yet implemented")

    from src.services.chat_engine import chat_engine

    # First query
    query1 = "What are actuators?"
    result1 = await chat_engine.query(query_text=query1)

    assert "actuator" in result1["answer_text"].lower()

    # Follow-up query (would need session context)
    query2 = "What types are there?"
    result2 = await chat_engine.query(query_text=query2)

    # Should reference actuator types based on context
    # (This requires conversation history implementation)

    print(f"✓ Multi-turn conversation test passed")


@pytest.mark.integration
def test_query_validation():
    """Test input validation for queries."""
    from src.utils.validators import validate_query_length, sanitize_query

    # Test valid queries
    assert validate_query_length("What are robots?")
    assert validate_query_length("A" * 4000)  # Max length

    # Test invalid queries
    assert not validate_query_length("")  # Empty
    assert not validate_query_length("A" * 5000)  # Too long

    # Test sanitization
    assert sanitize_query("  test  query  ") == "test query"
    assert "\x00" not in sanitize_query("test\x00query")

    print("✓ Query validation tests passed")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "integration"])
