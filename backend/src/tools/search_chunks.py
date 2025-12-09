"""
Search chunks tool for OpenAI Agents - performs semantic search in Qdrant.
"""
from typing import List, Dict, Any, Optional
from src.services.qdrant_client import qdrant_service
from src.services.embedder import embedder_service
from src.utils.logger import get_logger

logger = get_logger("search_chunks_tool")


async def search_chunks(
    query: str,
    selected_chunk_ids: Optional[List[str]] = None,
    top_k: int = 20
) -> List[Dict[str, Any]]:
    """
    Search for relevant chunks using semantic similarity.

    This tool is used by the OpenAI Agent to find relevant book chunks
    for answering user questions.

    Args:
        query: User's question or search query
        selected_chunk_ids: Optional list of chunk IDs to restrict search
        top_k: Number of results to return (default: 20)

    Returns:
        List of relevant chunks with metadata and relevance scores
    """
    try:
        logger.info(f"Searching for: '{query[:100]}...'")

        # Generate embedding for the query
        query_embedding = embedder_service.embed_text(query)

        # Search in Qdrant
        results = qdrant_service.search_vectors(
            query_vector=query_embedding,
            limit=top_k,
            chunk_ids_filter=selected_chunk_ids
        )

        logger.info(f"Found {len(results)} matching chunks")

        # Format results for the agent
        formatted_results = []
        for result in results:
            formatted_results.append({
                "chunk_id": result["chunk_id"],
                "text": result["payload"].get("text", ""),
                "relevance_score": result["score"],
                "metadata": {
                    "book_id": result["payload"].get("book_id"),
                    "chunk_index": result["payload"].get("chunk_index"),
                    "token_count": result["payload"].get("token_count")
                }
            })

        return formatted_results

    except Exception as e:
        logger.error(f"Error in search_chunks tool: {e}")
        return []


# Tool definition for OpenAI function calling
SEARCH_CHUNKS_TOOL = {
    "type": "function",
    "function": {
        "name": "search_chunks",
        "description": "Search for relevant text chunks from the book based on semantic similarity to answer the user's question. Use this to find information from the book.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query or question to find relevant chunks for"
                },
                "selected_chunk_ids": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Optional list of specific chunk IDs to search within (for selected-text mode)"
                },
                "top_k": {
                    "type": "integer",
                    "description": "Number of relevant chunks to return (default: 20)",
                    "default": 20
                }
            },
            "required": ["query"]
        }
    }
}
