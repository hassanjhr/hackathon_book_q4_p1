"""
ChatKit Server adapter for RAG chatbot.

Wraps the existing RAG service with ChatKit protocol for consistent chat interface.
Provides streaming responses compatible with ChatKit frontend components.
"""
import asyncio
from typing import AsyncGenerator, Dict, Any, Optional, List
from enum import Enum

from src.services.rag_service import rag_service
from src.services.chatkit_store import (
    chatkit_store,
    ThreadMetadata,
    ThreadItem,
    Page
)
from src.utils.logger import get_logger

logger = get_logger("chatkit_server")


# ============================================================================
# ChatKit Protocol Models
# ============================================================================

class EventType(str, Enum):
    """ChatKit stream event types."""
    THREAD_CREATED = "thread.created"
    THREAD_MESSAGE_DELTA = "thread.message.delta"
    THREAD_MESSAGE_COMPLETED = "thread.message.completed"
    THREAD_RUN_COMPLETED = "thread.run.completed"
    ERROR = "error"


class ThreadStreamEvent:
    """ChatKit streaming event."""

    def __init__(
        self,
        event: str,
        data: Dict[str, Any]
    ):
        self.event = event
        self.data = data

    def to_dict(self) -> Dict[str, Any]:
        return {
            "event": self.event,
            "data": self.data
        }

    def to_sse(self) -> str:
        """
        Format as Server-Sent Events (SSE) for streaming.

        Returns:
            SSE-formatted string
        """
        import json
        return f"event: {self.event}\ndata: {json.dumps(self.data)}\n\n"


# ============================================================================
# ChatKit Server Implementation
# ============================================================================

class RAGChatKitServer:
    """
    ChatKit server that wraps the existing RAG service.

    Provides ChatKit-compatible streaming interface while using the
    existing RAG backend for answer generation.

    Features:
    - Thread-based conversation management
    - Context7-enhanced RAG responses
    - Streaming SSE support
    - Source citation formatting
    - Multi-source context (textbook + library docs)
    """

    def __init__(self):
        self.store = chatkit_store
        self.rag = rag_service

    # ========================================================================
    # Main Response Generation
    # ========================================================================

    async def respond(
        self,
        thread_id: str,
        message_content: str,
        user_id: str = "anonymous",
        use_library_docs: bool = True,
        stream: bool = True
    ) -> AsyncGenerator[ThreadStreamEvent, None]:
        """
        Generate response to user message with streaming support.

        Args:
            thread_id: Thread identifier (creates new if doesn't exist)
            message_content: User's message content
            user_id: User identifier
            use_library_docs: Whether to use Context7 library docs
            stream: Whether to stream response (True) or return complete (False)

        Yields:
            ThreadStreamEvent objects for SSE streaming
        """
        try:
            # ================================================================
            # 1. Load or create thread
            # ================================================================
            thread = await self.store.load_thread(thread_id)

            if not thread:
                logger.info(f"Creating new thread {thread_id} for user {user_id}")
                thread = await self.store.create_thread(user_id=user_id)
                thread_id = thread.id

                # Emit thread created event
                if stream:
                    yield ThreadStreamEvent(
                        event=EventType.THREAD_CREATED,
                        data={
                            "thread_id": thread_id,
                            "user_id": user_id,
                            "created_at": thread.created_at.isoformat()
                        }
                    )

            # ================================================================
            # 2. Save user message
            # ================================================================
            user_message = ThreadItem.create_message(
                thread_id=thread_id,
                role="user",
                content=message_content
            )
            await self.store.add_thread_item(user_message)

            logger.info(f"Processing message in thread {thread_id}: {message_content[:100]}...")

            # ================================================================
            # 3. Generate RAG response
            # ================================================================
            rag_result = await self.rag.answer_question(
                question=message_content,
                use_library_docs=use_library_docs,
                include_sources=True
            )

            answer = rag_result["answer"]
            sources = rag_result.get("sources", [])

            # ================================================================
            # 4. Format response with citations
            # ================================================================
            formatted_answer = self._format_answer_with_citations(answer, sources)

            # ================================================================
            # 5. Stream or return complete response
            # ================================================================
            if stream:
                # Stream the answer in chunks (simulate streaming)
                chunk_size = 50  # characters per chunk
                for i in range(0, len(formatted_answer), chunk_size):
                    chunk = formatted_answer[i:i + chunk_size]

                    yield ThreadStreamEvent(
                        event=EventType.THREAD_MESSAGE_DELTA,
                        data={
                            "thread_id": thread_id,
                            "delta": chunk,
                            "role": "assistant"
                        }
                    )

                    # Small delay to simulate natural streaming
                    await asyncio.sleep(0.01)

                # Emit completion event
                yield ThreadStreamEvent(
                    event=EventType.THREAD_MESSAGE_COMPLETED,
                    data={
                        "thread_id": thread_id,
                        "content": formatted_answer,
                        "role": "assistant",
                        "sources": sources
                    }
                )

            # ================================================================
            # 6. Save assistant message
            # ================================================================
            assistant_message = ThreadItem.create_message(
                thread_id=thread_id,
                role="assistant",
                content=formatted_answer,
                metadata={
                    "sources": sources,
                    "use_library_docs": use_library_docs
                }
            )
            await self.store.add_thread_item(assistant_message)

            # ================================================================
            # 7. Emit run completed event
            # ================================================================
            if stream:
                yield ThreadStreamEvent(
                    event=EventType.THREAD_RUN_COMPLETED,
                    data={
                        "thread_id": thread_id,
                        "status": "completed",
                        "message_count": await self.store.get_thread_item_count(thread_id)
                    }
                )

        except Exception as e:
            logger.error(f"Error in ChatKit response generation: {e}")

            # Emit error event
            if stream:
                yield ThreadStreamEvent(
                    event=EventType.ERROR,
                    data={
                        "thread_id": thread_id,
                        "error": str(e),
                        "error_type": type(e).__name__
                    }
                )
            else:
                raise

    # ========================================================================
    # Non-Streaming Response (for compatibility)
    # ========================================================================

    async def generate_response(
        self,
        thread_id: str,
        message_content: str,
        user_id: str = "anonymous",
        use_library_docs: bool = True
    ) -> Dict[str, Any]:
        """
        Generate complete response (non-streaming).

        Args:
            thread_id: Thread identifier
            message_content: User's message content
            user_id: User identifier
            use_library_docs: Whether to use Context7 library docs

        Returns:
            Complete response with answer, sources, and metadata
        """
        # Collect all events from streaming response
        events = []
        async for event in self.respond(
            thread_id=thread_id,
            message_content=message_content,
            user_id=user_id,
            use_library_docs=use_library_docs,
            stream=True
        ):
            events.append(event)

        # Extract final response from completed event
        for event in reversed(events):
            if event.event == EventType.THREAD_MESSAGE_COMPLETED:
                return {
                    "thread_id": thread_id,
                    "answer": event.data["content"],
                    "sources": event.data.get("sources", []),
                    "role": event.data["role"]
                }

        # Fallback if no completion event
        return {
            "thread_id": thread_id,
            "answer": "Sorry, I couldn't generate a response.",
            "sources": [],
            "error": "No completion event received"
        }

    # ========================================================================
    # Thread Management
    # ========================================================================

    async def get_thread(self, thread_id: str) -> Optional[ThreadMetadata]:
        """
        Get thread metadata.

        Args:
            thread_id: Thread identifier

        Returns:
            ThreadMetadata if found, None otherwise
        """
        return await self.store.load_thread(thread_id)

    async def list_threads(
        self,
        user_id: str,
        limit: int = 50,
        offset: int = 0
    ) -> List[ThreadMetadata]:
        """
        List user's threads.

        Args:
            user_id: User identifier
            limit: Maximum number of threads
            offset: Pagination offset

        Returns:
            List of ThreadMetadata
        """
        return await self.store.list_threads(user_id, limit, offset)

    async def get_thread_messages(
        self,
        thread_id: str,
        limit: int = 50,
        cursor: Optional[str] = None
    ) -> Page:
        """
        Get messages in a thread with pagination.

        Args:
            thread_id: Thread identifier
            limit: Maximum number of messages
            cursor: Pagination cursor

        Returns:
            Page[ThreadItem] with messages
        """
        return await self.store.load_thread_items(thread_id, limit, cursor)

    async def delete_thread(self, thread_id: str) -> bool:
        """
        Delete a thread.

        Args:
            thread_id: Thread identifier

        Returns:
            True if deleted successfully
        """
        return await self.store.delete_thread(thread_id)

    # ========================================================================
    # Utility Methods
    # ========================================================================

    def _format_answer_with_citations(
        self,
        answer: str,
        sources: List[Dict[str, Any]]
    ) -> str:
        """
        Format answer with source citations.

        Args:
            answer: Generated answer text
            sources: List of source dictionaries

        Returns:
            Formatted answer with citations appended
        """
        if not sources:
            return answer

        # Group sources by type
        textbook_sources = [s for s in sources if s.get("type") == "textbook"]
        library_sources = [s for s in sources if s.get("type") == "library_docs"]

        citations = []

        # Add textbook citations
        if textbook_sources:
            citations.append("\n\n**📚 Textbook Sources:**")
            for i, source in enumerate(textbook_sources, 1):
                module = source.get("module", "Unknown Module")
                week = source.get("week", "Unknown Week")
                file = source.get("file", "Unknown File")
                score = source.get("relevance_score", 0)

                citations.append(
                    f"{i}. {module}, {week}: {file} (relevance: {score:.2f})"
                )

        # Add library documentation citations
        if library_sources:
            citations.append("\n\n**🔧 Library Documentation:**")
            for i, source in enumerate(library_sources, 1):
                library = source.get("library", "Documentation")
                citations.append(f"{i}. {library.upper()} API Documentation")

        return answer + "\n".join(citations)

    async def health_check(self) -> Dict[str, Any]:
        """
        Check ChatKit server health.

        Returns:
            Health status dictionary
        """
        try:
            # Check RAG service
            rag_health = await self.rag.health_check()

            # Check store connectivity (try to list threads)
            await self.store.list_threads("health_check", limit=1)

            return {
                "status": "healthy",
                "chatkit_server": "operational",
                "rag_service": rag_health.get("status"),
                "store": "operational"
            }

        except Exception as e:
            logger.error(f"ChatKit health check failed: {e}")
            return {
                "status": "unhealthy",
                "error": str(e)
            }


# ============================================================================
# Global Server Instance
# ============================================================================

chatkit_server = RAGChatKitServer()
