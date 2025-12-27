"""
RAG chatbot API endpoints.
"""
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any

from src.services.rag_service import rag_service
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
