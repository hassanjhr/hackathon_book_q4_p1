"""
ChatKit Store implementation using Postgres.

Implements OpenAI ChatKit's Store interface for thread and message persistence.
Based on ChatKit Store specification with ThreadMetadata, ThreadItem, and Page models.
"""
import uuid
from typing import Optional, List, Dict, Any
from datetime import datetime

from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("chatkit_store")


# ============================================================================
# Data Models (matching ChatKit specification)
# ============================================================================

class ThreadMetadata:
    """Thread metadata model."""

    def __init__(
        self,
        id: str,
        user_id: str,
        metadata: Optional[Dict[str, Any]] = None,
        created_at: Optional[datetime] = None,
        updated_at: Optional[datetime] = None
    ):
        self.id = id
        self.user_id = user_id
        self.metadata = metadata or {}
        self.created_at = created_at or datetime.utcnow()
        self.updated_at = updated_at or datetime.utcnow()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "user_id": self.user_id,
            "metadata": self.metadata,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ThreadMetadata":
        return cls(
            id=data["id"],
            user_id=data["user_id"],
            metadata=data.get("metadata", {}),
            created_at=data.get("created_at"),
            updated_at=data.get("updated_at"),
        )


class ThreadItem:
    """Thread item model (messages, tool calls, tasks, etc.)."""

    def __init__(
        self,
        id: str,
        thread_id: str,
        item_type: str,
        role: Optional[str] = None,
        content: Optional[str] = None,
        tool_name: Optional[str] = None,
        tool_arguments: Optional[Dict[str, Any]] = None,
        tool_call_id: Optional[str] = None,
        tool_result: Optional[str] = None,
        tool_result_call_id: Optional[str] = None,
        task_name: Optional[str] = None,
        task_status: Optional[str] = None,
        task_result: Optional[Dict[str, Any]] = None,
        workflow_name: Optional[str] = None,
        workflow_status: Optional[str] = None,
        workflow_steps: Optional[List[Dict[str, Any]]] = None,
        attachment_url: Optional[str] = None,
        attachment_mime_type: Optional[str] = None,
        attachment_size_bytes: Optional[int] = None,
        metadata: Optional[Dict[str, Any]] = None,
        created_at: Optional[datetime] = None,
        sequence_number: Optional[int] = None
    ):
        self.id = id
        self.thread_id = thread_id
        self.item_type = item_type
        self.role = role
        self.content = content
        self.tool_name = tool_name
        self.tool_arguments = tool_arguments
        self.tool_call_id = tool_call_id
        self.tool_result = tool_result
        self.tool_result_call_id = tool_result_call_id
        self.task_name = task_name
        self.task_status = task_status
        self.task_result = task_result
        self.workflow_name = workflow_name
        self.workflow_status = workflow_status
        self.workflow_steps = workflow_steps
        self.attachment_url = attachment_url
        self.attachment_mime_type = attachment_mime_type
        self.attachment_size_bytes = attachment_size_bytes
        self.metadata = metadata or {}
        self.created_at = created_at or datetime.utcnow()
        self.sequence_number = sequence_number

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "thread_id": self.thread_id,
            "item_type": self.item_type,
            "role": self.role,
            "content": self.content,
            "tool_name": self.tool_name,
            "tool_arguments": self.tool_arguments,
            "tool_call_id": self.tool_call_id,
            "tool_result": self.tool_result,
            "tool_result_call_id": self.tool_result_call_id,
            "task_name": self.task_name,
            "task_status": self.task_status,
            "task_result": self.task_result,
            "workflow_name": self.workflow_name,
            "workflow_status": self.workflow_status,
            "workflow_steps": self.workflow_steps,
            "attachment_url": self.attachment_url,
            "attachment_mime_type": self.attachment_mime_type,
            "attachment_size_bytes": self.attachment_size_bytes,
            "metadata": self.metadata,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "sequence_number": self.sequence_number,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ThreadItem":
        return cls(
            id=data["id"],
            thread_id=data["thread_id"],
            item_type=data["item_type"],
            role=data.get("role"),
            content=data.get("content"),
            tool_name=data.get("tool_name"),
            tool_arguments=data.get("tool_arguments"),
            tool_call_id=data.get("tool_call_id"),
            tool_result=data.get("tool_result"),
            tool_result_call_id=data.get("tool_result_call_id"),
            task_name=data.get("task_name"),
            task_status=data.get("task_status"),
            task_result=data.get("task_result"),
            workflow_name=data.get("workflow_name"),
            workflow_status=data.get("workflow_status"),
            workflow_steps=data.get("workflow_steps"),
            attachment_url=data.get("attachment_url"),
            attachment_mime_type=data.get("attachment_mime_type"),
            attachment_size_bytes=data.get("attachment_size_bytes"),
            metadata=data.get("metadata", {}),
            created_at=data.get("created_at"),
            sequence_number=data.get("sequence_number"),
        )

    @classmethod
    def create_message(
        cls,
        thread_id: str,
        role: str,
        content: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> "ThreadItem":
        """Helper to create a message-type thread item."""
        return cls(
            id=str(uuid.uuid4()),
            thread_id=thread_id,
            item_type="message",
            role=role,
            content=content,
            metadata=metadata
        )


class Page:
    """Pagination wrapper for thread items."""

    def __init__(
        self,
        items: List[ThreadItem],
        has_next: bool = False,
        next_cursor: Optional[str] = None
    ):
        self.items = items
        self.has_next = has_next
        self.next_cursor = next_cursor

    def to_dict(self) -> Dict[str, Any]:
        return {
            "items": [item.to_dict() for item in self.items],
            "has_next": self.has_next,
            "next_cursor": self.next_cursor,
        }


# ============================================================================
# ChatKit Store Implementation
# ============================================================================

class PostgresChatKitStore:
    """
    ChatKit Store implementation using Postgres.

    Implements the ChatKit Store interface for thread and message persistence.
    Supports ThreadMetadata, ThreadItem (messages, tool calls, tasks, etc.),
    and pagination with Page[ThreadItem].
    """

    def __init__(self):
        self.postgres = postgres_service

    # ========================================================================
    # Thread ID Generation
    # ========================================================================

    def generate_thread_id(self) -> str:
        """
        Generate a unique thread ID.

        Returns:
            Unique thread identifier (UUID v4)
        """
        return str(uuid.uuid4())

    # ========================================================================
    # Thread Metadata Operations
    # ========================================================================

    async def create_thread(
        self,
        user_id: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> ThreadMetadata:
        """
        Create a new thread.

        Args:
            user_id: User identifier
            metadata: Optional thread metadata

        Returns:
            Created ThreadMetadata
        """
        thread_id = self.generate_thread_id()

        query = """
            INSERT INTO chatkit_threads (id, user_id, metadata, created_at, updated_at)
            VALUES ($1, $2, $3, NOW(), NOW())
            RETURNING id, user_id, metadata, created_at, updated_at
        """

        result = await self.postgres.fetch_one(
            query,
            thread_id,
            user_id,
            metadata or {}
        )

        if not result:
            raise Exception(f"Failed to create thread for user {user_id}")

        logger.info(f"Created thread {thread_id} for user {user_id}")
        return ThreadMetadata.from_dict(dict(result))

    async def load_thread(self, thread_id: str) -> Optional[ThreadMetadata]:
        """
        Load thread metadata by ID.

        Args:
            thread_id: Thread identifier

        Returns:
            ThreadMetadata if found, None otherwise
        """
        query = """
            SELECT id, user_id, metadata, created_at, updated_at
            FROM chatkit_threads
            WHERE id = $1
        """

        result = await self.postgres.fetch_one(query, thread_id)

        if not result:
            logger.warning(f"Thread {thread_id} not found")
            return None

        return ThreadMetadata.from_dict(dict(result))

    async def save_thread(self, thread: ThreadMetadata) -> None:
        """
        Save (update) thread metadata.

        Args:
            thread: ThreadMetadata to save
        """
        query = """
            UPDATE chatkit_threads
            SET metadata = $2, updated_at = NOW()
            WHERE id = $1
        """

        await self.postgres.execute(query, thread.id, thread.metadata)
        logger.info(f"Updated thread {thread.id}")

    async def list_threads(
        self,
        user_id: str,
        limit: int = 50,
        offset: int = 0
    ) -> List[ThreadMetadata]:
        """
        List all threads for a user.

        Args:
            user_id: User identifier
            limit: Maximum number of threads to return
            offset: Pagination offset

        Returns:
            List of ThreadMetadata
        """
        query = """
            SELECT id, user_id, metadata, created_at, updated_at
            FROM chatkit_threads
            WHERE user_id = $1
            ORDER BY updated_at DESC
            LIMIT $2 OFFSET $3
        """

        results = await self.postgres.fetch_all(query, user_id, limit, offset)

        return [ThreadMetadata.from_dict(dict(row)) for row in results]

    async def delete_thread(self, thread_id: str) -> bool:
        """
        Delete a thread and all its items.

        Args:
            thread_id: Thread identifier

        Returns:
            True if deleted, False if not found
        """
        query = "DELETE FROM chatkit_threads WHERE id = $1"
        result = await self.postgres.execute(query, thread_id)

        deleted = result is not None
        if deleted:
            logger.info(f"Deleted thread {thread_id}")
        else:
            logger.warning(f"Thread {thread_id} not found for deletion")

        return deleted

    # ========================================================================
    # Thread Item Operations
    # ========================================================================

    async def add_thread_item(self, item: ThreadItem) -> ThreadItem:
        """
        Add an item to a thread.

        Args:
            item: ThreadItem to add

        Returns:
            Added ThreadItem with sequence_number
        """
        query = """
            INSERT INTO chatkit_thread_items (
                id, thread_id, item_type, role, content,
                tool_name, tool_arguments, tool_call_id,
                tool_result, tool_result_call_id,
                task_name, task_status, task_result,
                workflow_name, workflow_status, workflow_steps,
                attachment_url, attachment_mime_type, attachment_size_bytes,
                metadata, created_at
            ) VALUES (
                $1, $2, $3, $4, $5, $6, $7, $8, $9, $10,
                $11, $12, $13, $14, $15, $16, $17, $18, $19, $20, NOW()
            )
            RETURNING sequence_number, created_at
        """

        result = await self.postgres.fetch_one(
            query,
            item.id, item.thread_id, item.item_type, item.role, item.content,
            item.tool_name, item.tool_arguments, item.tool_call_id,
            item.tool_result, item.tool_result_call_id,
            item.task_name, item.task_status, item.task_result,
            item.workflow_name, item.workflow_status, item.workflow_steps,
            item.attachment_url, item.attachment_mime_type, item.attachment_size_bytes,
            item.metadata
        )

        if not result:
            raise Exception(f"Failed to add item to thread {item.thread_id}")

        item.sequence_number = result["sequence_number"]
        item.created_at = result["created_at"]

        logger.info(f"Added {item.item_type} item {item.id} to thread {item.thread_id}")
        return item

    async def load_thread_items(
        self,
        thread_id: str,
        limit: int = 50,
        cursor: Optional[str] = None
    ) -> Page:
        """
        Load thread items with pagination.

        Args:
            thread_id: Thread identifier
            limit: Maximum number of items to return
            cursor: Pagination cursor (sequence_number)

        Returns:
            Page[ThreadItem] with items and pagination info
        """
        # Parse cursor (sequence_number)
        after_sequence = int(cursor) if cursor else 0

        query = """
            SELECT
                id, thread_id, item_type, role, content,
                tool_name, tool_arguments, tool_call_id,
                tool_result, tool_result_call_id,
                task_name, task_status, task_result,
                workflow_name, workflow_status, workflow_steps,
                attachment_url, attachment_mime_type, attachment_size_bytes,
                metadata, created_at, sequence_number
            FROM chatkit_thread_items
            WHERE thread_id = $1 AND sequence_number > $2
            ORDER BY sequence_number ASC
            LIMIT $3
        """

        results = await self.postgres.fetch_all(query, thread_id, after_sequence, limit + 1)

        # Check if there are more items
        has_next = len(results) > limit
        items_data = results[:limit]

        items = [ThreadItem.from_dict(dict(row)) for row in items_data]

        # Next cursor is the last sequence_number
        next_cursor = str(items[-1].sequence_number) if items and has_next else None

        return Page(items=items, has_next=has_next, next_cursor=next_cursor)

    async def get_thread_item(self, item_id: str) -> Optional[ThreadItem]:
        """
        Get a specific thread item by ID.

        Args:
            item_id: Thread item identifier

        Returns:
            ThreadItem if found, None otherwise
        """
        query = """
            SELECT
                id, thread_id, item_type, role, content,
                tool_name, tool_arguments, tool_call_id,
                tool_result, tool_result_call_id,
                task_name, task_status, task_result,
                workflow_name, workflow_status, workflow_steps,
                attachment_url, attachment_mime_type, attachment_size_bytes,
                metadata, created_at, sequence_number
            FROM chatkit_thread_items
            WHERE id = $1
        """

        result = await self.postgres.fetch_one(query, item_id)

        if not result:
            return None

        return ThreadItem.from_dict(dict(result))

    async def update_thread_item(self, item: ThreadItem) -> None:
        """
        Update a thread item (e.g., update task status).

        Args:
            item: ThreadItem with updated fields
        """
        query = """
            UPDATE chatkit_thread_items
            SET
                content = $2,
                tool_result = $3,
                task_status = $4,
                task_result = $5,
                workflow_status = $6,
                workflow_steps = $7,
                metadata = $8
            WHERE id = $1
        """

        await self.postgres.execute(
            query,
            item.id,
            item.content,
            item.tool_result,
            item.task_status,
            item.task_result,
            item.workflow_status,
            item.workflow_steps,
            item.metadata
        )

        logger.info(f"Updated thread item {item.id}")

    # ========================================================================
    # Utility Methods
    # ========================================================================

    async def get_thread_item_count(self, thread_id: str) -> int:
        """
        Get the total number of items in a thread.

        Args:
            thread_id: Thread identifier

        Returns:
            Total item count
        """
        query = "SELECT COUNT(*) as count FROM chatkit_thread_items WHERE thread_id = $1"
        result = await self.postgres.fetch_one(query, thread_id)
        return result["count"] if result else 0

    async def cleanup_expired_cursors(self) -> int:
        """
        Clean up expired pagination cursors.

        Returns:
            Number of cursors deleted
        """
        query = "SELECT cleanup_expired_pagination_cursors()"
        result = await self.postgres.fetch_one(query)
        count = result["cleanup_expired_pagination_cursors"] if result else 0

        if count > 0:
            logger.info(f"Cleaned up {count} expired pagination cursors")

        return count


# ============================================================================
# Global Store Instance
# ============================================================================

chatkit_store = PostgresChatKitStore()
