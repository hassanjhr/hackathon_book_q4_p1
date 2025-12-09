"""
Get metadata tool for OpenAI Agents - retrieves detailed chunk information.
"""
from typing import Dict, Any, Optional
from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("get_metadata_tool")


async def get_metadata(chunk_id: str) -> Optional[Dict[str, Any]]:
    """
    Get detailed metadata for a specific chunk.

    This tool is used by the OpenAI Agent to retrieve full metadata
    about a chunk for citation purposes.

    Args:
        chunk_id: UUID of the chunk

    Returns:
        Dictionary with chunk metadata including full text and location info
    """
    try:
        logger.info(f"Fetching metadata for chunk: {chunk_id}")

        # Query Postgres for full chunk data
        chunk = await postgres_service.get_chunk(chunk_id)

        if not chunk:
            logger.warning(f"Chunk not found: {chunk_id}")
            return None

        # Format for the agent
        metadata = {
            "chunk_id": str(chunk["id"]),
            "full_text": chunk["text"],
            "chapter_num": chunk.get("chapter_num"),
            "chapter_title": chunk.get("chapter_title"),
            "page_num": chunk.get("page_num"),
            "section_heading": chunk.get("section_heading"),
            "token_count": chunk["token_count"],
            "location": f"Chapter {chunk.get('chapter_num', 'N/A')}, Page {chunk.get('page_num', 'N/A')}"
        }

        return metadata

    except Exception as e:
        logger.error(f"Error in get_metadata tool: {e}")
        return None


# Tool definition for OpenAI function calling
GET_METADATA_TOOL = {
    "type": "function",
    "function": {
        "name": "get_metadata",
        "description": "Get detailed metadata and full text for a specific chunk by its ID. Use this to get citation information and complete text.",
        "parameters": {
            "type": "object",
            "properties": {
                "chunk_id": {
                    "type": "string",
                    "description": "The UUID of the chunk to get metadata for"
                }
            },
            "required": ["chunk_id"]
        }
    }
}
