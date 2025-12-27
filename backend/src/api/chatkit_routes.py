"""
ChatKit API routes for RAG chatbot.

FastAPI endpoints providing ChatKit-compatible chat interface with:
- Server-Sent Events (SSE) streaming
- Thread-based conversation management
- Context7-enhanced RAG responses
- Source citations
"""
from fastapi import APIRouter, HTTPException, status
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
import json

from src.services.chatkit_server import chatkit_server
from src.utils.logger import get_logger

logger = get_logger("chatkit_routes")

router = APIRouter()


# ============================================================================
# Request/Response Models
# ============================================================================

class ChatMessage(BaseModel):
    """Chat message model."""
    role: str = Field(..., description="Message role: user, assistant, system")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's message"
    )
    thread_id: Optional[str] = Field(
        None,
        description="Thread ID (creates new if not provided)"
    )
    user_id: str = Field(
        "anonymous",
        description="User identifier"
    )
    use_library_docs: bool = Field(
        True,
        description="Whether to enhance with Context7 library documentation"
    )
    stream: bool = Field(
        True,
        description="Whether to stream response with SSE"
    )


class ChatResponse(BaseModel):
    """Response model for chat endpoint (non-streaming)."""

    thread_id: str = Field(..., description="Thread identifier")
    answer: str = Field(..., description="AI-generated answer")
    sources: List[Dict[str, Any]] = Field(
        default=[],
        description="Source citations"
    )
    role: str = Field(default="assistant", description="Response role")
    error: Optional[str] = Field(None, description="Error message if any")


class ThreadMetadataResponse(BaseModel):
    """Thread metadata response."""

    id: str
    user_id: str
    metadata: Dict[str, Any]
    created_at: str
    updated_at: str


class ThreadListResponse(BaseModel):
    """Thread list response."""

    threads: List[ThreadMetadataResponse]
    count: int


class ThreadMessagesResponse(BaseModel):
    """Thread messages response with pagination."""

    thread_id: str
    messages: List[Dict[str, Any]]
    has_next: bool
    next_cursor: Optional[str] = None


class HealthResponse(BaseModel):
    """Health check response."""

    status: str
    chatkit_server: Optional[str] = None
    rag_service: Optional[str] = None
    store: Optional[str] = None
    error: Optional[str] = None


# ============================================================================
# Chat Endpoints
# ============================================================================

@router.post("/chatkit/chat", status_code=status.HTTP_200_OK)
async def chat(request: ChatRequest):
    """
    ChatKit chat endpoint with streaming support.

    **Streaming Mode (stream=true)**:
    Returns Server-Sent Events (SSE) stream with real-time response chunks.
    Event types:
    - `thread.created`: New thread created
    - `thread.message.delta`: Response chunk
    - `thread.message.completed`: Complete response
    - `thread.run.completed`: Processing completed
    - `error`: Error occurred

    **Non-Streaming Mode (stream=false)**:
    Returns complete response as JSON.

    **Features**:
    - Thread-based conversation history
    - Context7 library documentation enhancement
    - Source citations (textbook + library docs)
    - Graceful error handling

    Args:
        request: ChatRequest with message and options

    Returns:
        StreamingResponse (SSE) or ChatResponse (JSON)
    """
    try:
        logger.info(f"ChatKit request: user={request.user_id}, thread={request.thread_id}, stream={request.stream}")

        # Validate message
        if not request.message.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message cannot be empty"
            )

        # Generate thread ID if not provided
        if not request.thread_id:
            request.thread_id = chatkit_server.store.generate_thread_id()
            logger.info(f"Generated new thread ID: {request.thread_id}")

        # ====================================================================
        # Streaming Response (SSE)
        # ====================================================================
        if request.stream:
            async def event_generator():
                """Generate SSE events from ChatKit server."""
                try:
                    async for event in chatkit_server.respond(
                        thread_id=request.thread_id,
                        message_content=request.message,
                        user_id=request.user_id,
                        use_library_docs=request.use_library_docs,
                        stream=True
                    ):
                        # Convert event to SSE format
                        yield event.to_sse()

                except Exception as e:
                    logger.error(f"Error in SSE stream: {e}")
                    # Send error event
                    error_event = {
                        "event": "error",
                        "data": json.dumps({
                            "error": str(e),
                            "error_type": type(e).__name__
                        })
                    }
                    yield f"event: {error_event['event']}\ndata: {error_event['data']}\n\n"

            return StreamingResponse(
                event_generator(),
                media_type="text/event-stream",
                headers={
                    "Cache-Control": "no-cache",
                    "Connection": "keep-alive",
                    "X-Accel-Buffering": "no"  # Disable nginx buffering
                }
            )

        # ====================================================================
        # Non-Streaming Response (JSON)
        # ====================================================================
        else:
            result = await chatkit_server.generate_response(
                thread_id=request.thread_id,
                message_content=request.message,
                user_id=request.user_id,
                use_library_docs=request.use_library_docs
            )

            return ChatResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"ChatKit chat error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process chat request: {str(e)}"
        )


# ============================================================================
# Thread Management Endpoints
# ============================================================================

@router.get("/chatkit/threads/{user_id}", response_model=ThreadListResponse)
async def list_user_threads(
    user_id: str,
    limit: int = 50,
    offset: int = 0
):
    """
    List all threads for a user.

    Args:
        user_id: User identifier
        limit: Maximum number of threads to return (default: 50, max: 100)
        offset: Pagination offset (default: 0)

    Returns:
        ThreadListResponse with threads and count
    """
    try:
        # Validate limit
        if limit > 100:
            limit = 100

        threads = await chatkit_server.list_threads(user_id, limit, offset)

        return ThreadListResponse(
            threads=[
                ThreadMetadataResponse(**thread.to_dict())
                for thread in threads
            ],
            count=len(threads)
        )

    except Exception as e:
        logger.error(f"Error listing threads for user {user_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to list threads: {str(e)}"
        )


@router.get("/chatkit/threads/{thread_id}/messages", response_model=ThreadMessagesResponse)
async def get_thread_messages(
    thread_id: str,
    limit: int = 50,
    cursor: Optional[str] = None
):
    """
    Get messages in a thread with pagination.

    Args:
        thread_id: Thread identifier
        limit: Maximum number of messages (default: 50, max: 100)
        cursor: Pagination cursor (sequence_number from previous page)

    Returns:
        ThreadMessagesResponse with messages and pagination info
    """
    try:
        # Validate limit
        if limit > 100:
            limit = 100

        # Verify thread exists
        thread = await chatkit_server.get_thread(thread_id)
        if not thread:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Thread {thread_id} not found"
            )

        # Get messages with pagination
        page = await chatkit_server.get_thread_messages(thread_id, limit, cursor)

        return ThreadMessagesResponse(
            thread_id=thread_id,
            messages=[item.to_dict() for item in page.items],
            has_next=page.has_next,
            next_cursor=page.next_cursor
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting messages for thread {thread_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get messages: {str(e)}"
        )


@router.delete("/chatkit/threads/{thread_id}", status_code=status.HTTP_200_OK)
async def delete_thread(thread_id: str):
    """
    Delete a thread and all its messages.

    Args:
        thread_id: Thread identifier

    Returns:
        Success message
    """
    try:
        deleted = await chatkit_server.delete_thread(thread_id)

        if not deleted:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Thread {thread_id} not found"
            )

        return {
            "status": "success",
            "message": f"Thread {thread_id} deleted successfully"
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting thread {thread_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete thread: {str(e)}"
        )


# ============================================================================
# Health Check Endpoint
# ============================================================================

@router.get("/chatkit/health", response_model=HealthResponse)
async def health_check():
    """
    Check ChatKit service health.

    Tests:
    - ChatKit server connectivity
    - RAG service health
    - Database store connectivity

    Returns:
        HealthResponse with service statuses
    """
    try:
        result = await chatkit_server.health_check()
        return HealthResponse(**result)

    except Exception as e:
        logger.error(f"ChatKit health check failed: {e}")
        return HealthResponse(
            status="unhealthy",
            error=str(e)
        )


# ============================================================================
# Test Endpoint
# ============================================================================

@router.post("/chatkit/test", status_code=status.HTTP_200_OK)
async def test_endpoint(
    question: str = "What is Physical AI?",
    stream: bool = False
):
    """
    Simple test endpoint for quick verification.

    Args:
        question: Test question (default: "What is Physical AI?")
        stream: Whether to test streaming (default: False)

    Returns:
        Test result with answer and metadata
    """
    try:
        # Generate thread ID
        thread_id = chatkit_server.store.generate_thread_id()

        if stream:
            # Test streaming mode
            events = []
            async for event in chatkit_server.respond(
                thread_id=thread_id,
                message_content=question,
                user_id="test-user",
                use_library_docs=True,
                stream=True
            ):
                events.append(event.to_dict())

            return {
                "status": "success",
                "mode": "streaming",
                "test_question": question,
                "thread_id": thread_id,
                "events_count": len(events),
                "events": events
            }

        else:
            # Test non-streaming mode
            result = await chatkit_server.generate_response(
                thread_id=thread_id,
                message_content=question,
                user_id="test-user",
                use_library_docs=True
            )

            return {
                "status": "success",
                "mode": "non-streaming",
                "test_question": question,
                "thread_id": thread_id,
                "answer": result["answer"],
                "sources_count": len(result.get("sources", [])),
                "sources": result.get("sources", [])
            }

    except Exception as e:
        logger.error(f"ChatKit test endpoint failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Test failed: {str(e)}"
        )
