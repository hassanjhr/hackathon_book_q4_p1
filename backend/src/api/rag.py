"""
RAG chatbot API endpoints.
"""
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
import os

from src.services.rag_service import rag_service
from src.services.qdrant_client import qdrant_service
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("rag_api")

router = APIRouter()


class ChatRequest(BaseModel):
    """Request model for RAG chat endpoint."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's question about the textbook content"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=10000,
        description="Optional text selected by user for targeted answering"
    )
    top_k: Optional[int] = Field(
        None,
        ge=1,
        le=50,
        description="Number of chunks to retrieve (defaults to config)"
    )
    include_sources: bool = Field(
        True,
        description="Whether to include source citations in response"
    )
    use_library_docs: bool = Field(
        True,
        description="Whether to enhance with Context7 library documentation"
    )


class Source(BaseModel):
    """Source citation model."""

    module: str = Field(..., description="Module name (e.g., 'Module 0')")
    week: str = Field(..., description="Week name (e.g., 'Week 1')")
    file: str = Field(..., description="Source file name")
    relevance_score: Optional[float] = Field(None, description="Relevance score (0-1)")


class ChatResponse(BaseModel):
    """Response model for RAG chat endpoint."""

    answer: str = Field(..., description="AI-generated answer")
    sources: List[Dict[str, Any]] = Field(
        default=[],
        description="List of source citations"
    )
    error: Optional[str] = Field(None, description="Error message if any")


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str
    embedder: Optional[str] = None
    vector_db: Optional[str] = None
    database: Optional[str] = None
    error: Optional[str] = None


@router.post("/rag/chat", response_model=ChatResponse, status_code=status.HTTP_200_OK)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Answer questions using RAG on textbook content, enhanced with Context7 library docs.

    This endpoint:
    1. Accepts a user question and optional selected text
    2. Detects if question involves a library (PyTorch, ROS2, etc.)
    3. Generates embedding for the query
    4. Searches Qdrant vector database for relevant textbook chunks
    5. If library detected, fetches documentation from Context7
    6. Merges textbook + library contexts
    7. Generates grounded answer using OpenAI
    8. Returns answer with source citations from both sources

    **Special features**:
    - If `selected_text` is provided, answers based ONLY on that text
    - If `use_library_docs=True`, enhances with Context7 library documentation
    - Gracefully falls back to textbook-only if Context7 fails

    Args:
        request: ChatRequest with question and optional parameters

    Returns:
        ChatResponse with answer and sources

    Raises:
        HTTPException: If processing fails
    """
    try:
        logger.info(f"Received chat request: {request.question[:100]}...")

        # Validate request
        if not request.question.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Question cannot be empty"
            )

        # Process question through RAG service
        result = await rag_service.answer_question(
            question=request.question,
            selected_text=request.selected_text,
            top_k=request.top_k,
            include_sources=request.include_sources,
            use_library_docs=request.use_library_docs
        )

        return ChatResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process question: {str(e)}"
        )


@router.get("/rag/health", response_model=HealthResponse, status_code=status.HTTP_200_OK)
async def health() -> HealthResponse:
    """
    Check RAG service health.

    Tests:
    - Embedder service (OpenAI API)
    - Vector database (Qdrant)
    - Metadata database (Postgres)

    Returns:
        HealthResponse with service statuses
    """
    try:
        result = await rag_service.health_check()
        return HealthResponse(**result)
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthResponse(
            status="unhealthy",
            error=str(e)
        )


@router.post("/rag/test", status_code=status.HTTP_200_OK)
async def test_endpoint(question: str = "What is Physical AI?"):
    """
    Simple test endpoint for quick verification.

    Args:
        question: Test question (default: "What is Physical AI?")

    Returns:
        Test result with answer and metadata
    """
    try:
        result = await rag_service.answer_question(
            question=question,
            include_sources=True
        )

        return {
            "status": "success",
            "test_question": question,
            "answer": result["answer"],
            "sources_count": len(result.get("sources", [])),
            "sources": result.get("sources", [])
        }

    except Exception as e:
        logger.error(f"Test endpoint failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Test failed: {str(e)}"
        )


@router.get("/rag/debug", status_code=status.HTTP_200_OK)
async def debug_qdrant():
    """
    Debug endpoint to diagnose Qdrant connection issues.

    Returns detailed diagnostic information including:
    - Environment variable values
    - Settings loaded by pydantic
    - Qdrant service status
    - Connection test results

    Use this to troubleshoot deployment issues.
    """
    try:
        # Get raw environment variables
        qdrant_url_env = os.getenv("QDRANT_URL", "")
        qdrant_api_key_env = os.getenv("QDRANT_API_KEY", "")

        diagnostics = {
            "environment_variables": {
                "QDRANT_URL": qdrant_url_env if qdrant_url_env else "NOT SET",
                "QDRANT_URL_LENGTH": len(qdrant_url_env) if qdrant_url_env else 0,
                "QDRANT_API_KEY": "SET (" + str(len(qdrant_api_key_env)) + " chars)" if qdrant_api_key_env else "NOT SET",
                "QDRANT_COLLECTION_NAME": os.getenv("QDRANT_COLLECTION_NAME", "NOT SET")
            },
            "settings_loaded_by_pydantic": {
                "qdrant_url": settings.qdrant_url if settings.qdrant_url else "NOT SET",
                "qdrant_url_length": len(settings.qdrant_url) if settings.qdrant_url else 0,
                "has_api_key": bool(settings.qdrant_api_key),
                "api_key_length": len(settings.qdrant_api_key) if settings.qdrant_api_key else 0,
                "collection_name": settings.qdrant_collection_name
            },
            "qdrant_service_status": {
                "is_available": qdrant_service.is_available,
                "client_configured": qdrant_service.client is not None,
                "collection_name": qdrant_service.collection_name
            }
        }

        # Try to connect and get collections
        if qdrant_service.is_available:
            try:
                from qdrant_client import QdrantClient

                # Test with current configuration
                test_client = QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key,
                    prefer_grpc=False,
                    https=True,
                    timeout=10
                )

                collections = test_client.get_collections()
                diagnostics["qdrant_connection_test"] = {
                    "status": "SUCCESS",
                    "collections": [c.name for c in collections.collections],
                    "collection_count": len(collections.collections),
                    "target_collection_exists": any(c.name == settings.qdrant_collection_name for c in collections.collections)
                }
            except Exception as e:
                diagnostics["qdrant_connection_test"] = {
                    "status": "FAILED",
                    "error": str(e),
                    "error_type": type(e).__name__
                }
        else:
            diagnostics["qdrant_connection_test"] = {
                "status": "SKIPPED",
                "reason": "Qdrant service marked as unavailable during initialization"
            }

        # Add recommendations based on diagnostics
        recommendations = []

        if not qdrant_url_env:
            recommendations.append("❌ QDRANT_URL environment variable is not set")
        elif qdrant_url_env != settings.qdrant_url:
            recommendations.append(f"⚠️  Environment variable QDRANT_URL='{qdrant_url_env}' doesn't match settings.qdrant_url='{settings.qdrant_url}'")

        if not qdrant_api_key_env:
            recommendations.append("❌ QDRANT_API_KEY environment variable is not set")

        if not qdrant_service.is_available:
            recommendations.append("❌ Qdrant service failed to initialize - check connection parameters")

        if diagnostics.get("qdrant_connection_test", {}).get("status") == "FAILED":
            error_msg = diagnostics["qdrant_connection_test"].get("error", "")
            if "Name or service not known" in error_msg:
                recommendations.append(f"🔥 DNS resolution failed for '{settings.qdrant_url}' - verify URL format")
                recommendations.append("Try: Remove https:// prefix if present, ensure port :6333 is included")
            elif "timeout" in error_msg.lower():
                recommendations.append("⏱️  Connection timeout - check firewall/network restrictions")

        if recommendations:
            diagnostics["recommendations"] = recommendations
        else:
            diagnostics["recommendations"] = ["✅ All checks passed!"]

        return diagnostics

    except Exception as e:
        logger.error(f"Error in debug endpoint: {e}")
        return {
            "error": str(e),
            "error_type": type(e).__name__,
            "recommendations": ["Failed to generate diagnostics - check server logs"]
        }
