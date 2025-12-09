"""
Integration tests for API endpoints.

Tests HTTP endpoints with FastAPI test client.
"""
import pytest
from httpx import AsyncClient
from pathlib import Path


@pytest.mark.asyncio
async def test_health_check_endpoint():
    """Test that the health check endpoint works."""
    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/health")

        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "healthy"
        assert "services" in data

    print("✓ Health check endpoint test passed")


@pytest.mark.asyncio
async def test_root_endpoint():
    """Test the root endpoint."""
    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/")

        assert response.status_code == 200

        data = response.json()
        assert "message" in data
        assert data["status"] == "healthy"

    print("✓ Root endpoint test passed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_ingest_endpoint():
    """Test the ingestion endpoint."""
    pytest.skip("Integration test - requires actual services")

    from src.main import app

    # Prepare test file
    test_file_path = Path(__file__).parent.parent / "fixtures" / "sample_book.txt"

    async with AsyncClient(app=app, base_url="http://test") as client:
        with open(test_file_path, 'rb') as f:
            files = {"file": ("sample_book.txt", f, "text/plain")}
            data = {
                "title": "Test Book",
                "author": "Test Author",
                "version": "1.0"
            }

            response = await client.post("/api/ingest", files=files, data=data)

        assert response.status_code == 200

        result = response.json()
        assert "job_id" in result
        assert result["status"] in ["completed", "processing"]
        assert result["total_chunks"] > 0

    print("✓ Ingest endpoint test passed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_ingestion_status_endpoint():
    """Test the ingestion status endpoint."""
    pytest.skip("Integration test - requires actual services")

    from src.main import app
    from uuid import uuid4

    # First create an ingestion job
    test_file_path = Path(__file__).parent.parent / "fixtures" / "sample_book.txt"

    async with AsyncClient(app=app, base_url="http://test") as client:
        # Upload file
        with open(test_file_path, 'rb') as f:
            files = {"file": ("sample_book.txt", f, "text/plain")}
            data = {"title": "Test Book", "author": "Test"}

            ingest_response = await client.post("/api/ingest", files=files, data=data)

        job_id = ingest_response.json()["job_id"]

        # Check status
        status_response = await client.get(f"/api/ingest/status/{job_id}")

        assert status_response.status_code == 200

        status_data = status_response.json()
        assert "status" in status_data
        assert status_data["status"] in ["completed", "processing", "failed"]

    print("✓ Ingestion status endpoint test passed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_query_endpoint():
    """Test the query endpoint."""
    pytest.skip("Integration test - requires actual services and ingested content")

    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        query_data = {
            "query_text": "What are the main components of a humanoid robot?"
        }

        response = await client.post(
            "/api/query",
            json=query_data
        )

        assert response.status_code == 200

        result = response.json()
        assert "answer_text" in result
        assert "citations" in result
        assert "processing_time_ms" in result
        assert "mode" in result

        assert len(result["answer_text"]) > 0
        assert result["mode"] == "full-book"

    print("✓ Query endpoint test passed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_query_endpoint_with_selected_chunks():
    """Test query endpoint with selected chunk IDs (selected-text mode)."""
    pytest.skip("Integration test - requires actual services")

    from src.main import app
    from src.services.postgres_client import postgres_service

    # Get some chunk IDs from database
    async with postgres_service.pool.acquire() as conn:
        chunks = await conn.fetch("SELECT id FROM chunks LIMIT 2")
        chunk_ids = [str(chunk['id']) for chunk in chunks]

    async with AsyncClient(app=app, base_url="http://test") as client:
        query_data = {
            "query_text": "Explain this",
            "selected_chunk_ids": chunk_ids
        }

        response = await client.post("/api/query", json=query_data)

        assert response.status_code == 200

        result = response.json()
        assert result["mode"] == "selected-text"

    print("✓ Query with selected chunks test passed")


@pytest.mark.asyncio
async def test_query_validation():
    """Test query validation logic."""
    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        # Test empty query
        response = await client.post(
            "/api/query",
            json={"query_text": ""}
        )
        assert response.status_code == 400

        # Test too long query
        response = await client.post(
            "/api/query",
            json={"query_text": "A" * 5000}
        )
        assert response.status_code == 400

    print("✓ Query validation test passed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_get_chunk_endpoint():
    """Test the get chunk by ID endpoint."""
    pytest.skip("Integration test - requires actual database")

    from src.main import app
    from src.services.postgres_client import postgres_service

    # Get a real chunk ID
    async with postgres_service.pool.acquire() as conn:
        chunk = await conn.fetchrow("SELECT id FROM chunks LIMIT 1")
        chunk_id = str(chunk['id'])

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get(f"/api/doc/{chunk_id}")

        assert response.status_code == 200

        data = response.json()
        assert "chunk_id" in data
        assert "text" in data
        assert "metadata" in data

        assert data["chunk_id"] == chunk_id

    print("✓ Get chunk endpoint test passed")


@pytest.mark.asyncio
async def test_get_chunk_not_found():
    """Test get chunk endpoint with invalid ID."""
    pytest.skip("Requires database connection")

    from src.main import app
    from uuid import uuid4

    fake_id = str(uuid4())

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get(f"/api/doc/{fake_id}")

        assert response.status_code == 404

    print("✓ Get chunk not found test passed")


@pytest.mark.asyncio
async def test_cors_headers():
    """Test that CORS headers are properly configured."""
    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.options(
            "/api/query",
            headers={"Origin": "http://localhost:3000"}
        )

        # CORS should allow the request
        assert response.status_code in [200, 204]

    print("✓ CORS headers test passed")


@pytest.mark.asyncio
async def test_error_handling():
    """Test API error handling."""
    from src.main import app

    async with AsyncClient(app=app, base_url="http://test") as client:
        # Test invalid JSON
        response = await client.post(
            "/api/query",
            content=b"invalid json",
            headers={"Content-Type": "application/json"}
        )

        assert response.status_code == 422  # Unprocessable entity

    print("✓ Error handling test passed")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_concurrent_requests():
    """Test handling of concurrent requests."""
    pytest.skip("Integration test - requires actual services")

    import asyncio
    from src.main import app

    async def make_query(client, query_text):
        response = await client.post(
            "/api/query",
            json={"query_text": query_text}
        )
        return response.status_code

    async with AsyncClient(app=app, base_url="http://test") as client:
        # Make 10 concurrent requests
        queries = [f"Test query {i}" for i in range(10)]

        tasks = [make_query(client, q) for q in queries]
        results = await asyncio.gather(*tasks)

        # All should succeed
        assert all(status == 200 for status in results)

    print("✓ Concurrent requests test passed")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
