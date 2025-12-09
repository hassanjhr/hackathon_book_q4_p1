"""
Input validation helpers for the RAG chatbot.
"""
import re
from typing import List, Optional
from uuid import UUID


def validate_chunk_id(chunk_id: str) -> bool:
    """
    Validate chunk ID format (UUID).

    Args:
        chunk_id: Chunk ID to validate

    Returns:
        True if valid UUID format, False otherwise
    """
    try:
        UUID(chunk_id)
        return True
    except (ValueError, AttributeError):
        return False


def validate_query_length(query: str, max_length: int = 4000) -> bool:
    """
    Validate query text length.

    Args:
        query: Query text to validate
        max_length: Maximum allowed length (default: 4000)

    Returns:
        True if query is within length limit, False otherwise
    """
    return 0 < len(query.strip()) <= max_length


def validate_token_count(count: int, max_count: int = 512) -> bool:
    """
    Validate token count is within acceptable range.

    Args:
        count: Token count to validate
        max_count: Maximum allowed tokens (default: 512)

    Returns:
        True if count is valid, False otherwise
    """
    return 0 < count <= max_count


def validate_chunk_ids(chunk_ids: Optional[List[str]]) -> bool:
    """
    Validate list of chunk IDs.

    Args:
        chunk_ids: List of chunk IDs to validate

    Returns:
        True if all chunk IDs are valid UUIDs, False otherwise
    """
    if chunk_ids is None:
        return True

    if not isinstance(chunk_ids, list):
        return False

    if not chunk_ids:  # Empty list
        return False

    return all(validate_chunk_id(cid) for cid in chunk_ids)


def sanitize_query(query: str) -> str:
    """
    Sanitize query text by removing dangerous characters and normalizing whitespace.

    Args:
        query: Raw query text

    Returns:
        Sanitized query text
    """
    # Remove null bytes and control characters
    query = re.sub(r'[\x00-\x1f\x7f]', '', query)

    # Normalize whitespace
    query = ' '.join(query.split())

    return query.strip()
