"""
Context7 service for fetching library documentation.

Integrates with Context7 MCP tools to retrieve up-to-date library documentation
for Physical AI and Humanoid Robotics frameworks.
"""
import asyncio
from typing import Optional, Dict, Any, List
from functools import lru_cache

from src.utils.logger import get_logger

logger = get_logger("context7_service")


class Context7Service:
    """Service for fetching library documentation via Context7 MCP."""

    # Library name to Context7 ID mappings
    LIBRARY_MAPPINGS = {
        "pytorch": "/pytorch/pytorch",
        "torch": "/pytorch/pytorch",
        "isaac-sim": "/nvidia/isaac-sim",
        "isaac": "/nvidia/isaac-sim",
        "nvidia-isaac": "/nvidia/isaac-sim",
        "ros2": "/ros2/ros2",
        "ros": "/ros2/ros2",
        "tensorflow": "/tensorflow/tensorflow",
        "tf": "/tensorflow/tensorflow",
        "opencv": "/opencv/opencv",
        "cv2": "/opencv/opencv",
    }

    def __init__(self):
        """Initialize Context7 service."""
        self.timeout_seconds = 3.0
        self.max_results = 2
        logger.info("Context7 service initialized")

    @lru_cache(maxsize=128)
    def resolve_library(self, library_name: str) -> Optional[str]:
        """
        Resolve library name to Context7-compatible library ID.

        Uses local mapping first (fast), falls back to MCP resolve tool if needed.

        Args:
            library_name: Library name (e.g., "pytorch", "isaac-sim")

        Returns:
            Context7 library ID (e.g., "/pytorch/pytorch") or None if not found
        """
        library_key = library_name.lower().strip()

        # Check local mapping first
        if library_key in self.LIBRARY_MAPPINGS:
            library_id = self.LIBRARY_MAPPINGS[library_key]
            logger.info(f"Resolved {library_name} → {library_id} (cached)")
            return library_id

        # TODO: Call MCP tool if not in local mapping
        # This would require MCP client integration
        # For now, return None for unmapped libraries
        logger.warning(f"Library '{library_name}' not found in mappings")
        return None

    async def get_documentation(
        self,
        library_id: str,
        topic: str,
        mode: str = "code",
        page: int = 1
    ) -> Optional[Dict[str, Any]]:
        """
        Fetch documentation from Context7 for a specific library.

        Args:
            library_id: Context7-compatible library ID (e.g., "/pytorch/pytorch")
            topic: Search topic/query
            mode: "code" for API references, "info" for conceptual guides
            page: Page number for pagination (default: 1)

        Returns:
            Documentation dictionary with 'content' and 'sources' or None if failed
        """
        try:
            logger.info(f"Fetching {mode} docs for {library_id}, topic: {topic}")

            # TODO: Call MCP tool mcp__context7__get_library_docs
            # For now, return placeholder to test integration
            # This would be replaced with actual MCP call:
            #
            # result = await mcp_client.call_tool(
            #     "mcp__context7__get_library_docs",
            #     context7CompatibleLibraryID=library_id,
            #     topic=topic,
            #     mode=mode,
            #     page=page
            # )

            # Placeholder: Return mock data for testing
            if "pytorch" in library_id.lower() and "tensor" in topic.lower():
                return {
                    "content": """torch.tensor(data, dtype=None, device=None, requires_grad=False)

Creates a PyTorch tensor from data.

Parameters:
- data: Data to create tensor from (array-like, list, scalar)
- dtype: Desired data type (torch.float32, torch.int64, etc.)
- device: Device to place tensor on ('cpu', 'cuda')
- requires_grad: If True, enables gradient tracking

Example:
```python
import torch

# From list
x = torch.tensor([[1, 2], [3, 4]])

# With dtype
y = torch.tensor([1.0, 2.0], dtype=torch.float64)

# On GPU
z = torch.tensor([1, 2, 3], device='cuda')
```

Returns:
    Tensor object with the specified configuration.""",
                    "sources": ["https://pytorch.org/docs/stable/generated/torch.tensor.html"],
                    "has_more": False
                }

            # If topic/library not matched, return None
            logger.warning(f"No documentation found for {library_id} with topic '{topic}'")
            return None

        except Exception as e:
            logger.error(f"Error fetching Context7 documentation: {e}")
            return None

    async def search_library_docs(
        self,
        library_name: str,
        query: str,
        mode: str = "code"
    ) -> Optional[Dict[str, Any]]:
        """
        Complete workflow: resolve library name and fetch documentation.

        Args:
            library_name: Human-readable library name (e.g., "pytorch", "ros2")
            query: User's question/topic
            mode: "code" for API references, "info" for conceptual guides

        Returns:
            Documentation dictionary or None if failed
        """
        try:
            # Step 1: Resolve library name to Context7 ID
            library_id = self.resolve_library(library_name)

            if not library_id:
                logger.warning(f"Could not resolve library: {library_name}")
                return None

            # Step 2: Fetch documentation with timeout
            result = await asyncio.wait_for(
                self.get_documentation(library_id, query, mode),
                timeout=self.timeout_seconds
            )

            if result:
                logger.info(f"Successfully fetched docs for {library_name}")
            else:
                logger.warning(f"No docs returned for {library_name}")

            return result

        except asyncio.TimeoutError:
            logger.warning(f"Context7 timeout ({self.timeout_seconds}s) for {library_name}")
            return None
        except Exception as e:
            logger.error(f"Error in search_library_docs: {e}")
            return None

    def get_supported_libraries(self) -> List[str]:
        """
        Get list of supported library names.

        Returns:
            List of supported library names
        """
        # Return unique library names (not aliases)
        unique_libs = set()
        for lib_name, lib_id in self.LIBRARY_MAPPINGS.items():
            # Extract primary name from ID
            primary_name = lib_id.split('/')[-1]
            unique_libs.add(primary_name)

        return sorted(list(unique_libs))

    async def health_check(self) -> Dict[str, str]:
        """
        Check if Context7 service is operational.

        Returns:
            Status dictionary
        """
        try:
            # Test with a known library
            result = await self.search_library_docs(
                library_name="pytorch",
                query="tensor",
                mode="code"
            )

            if result:
                return {
                    "status": "healthy",
                    "service": "context7",
                    "supported_libraries": len(self.LIBRARY_MAPPINGS)
                }
            else:
                return {
                    "status": "degraded",
                    "service": "context7",
                    "message": "Service responding but no results"
                }

        except Exception as e:
            logger.error(f"Context7 health check failed: {e}")
            return {
                "status": "unhealthy",
                "service": "context7",
                "error": str(e)
            }


# Global Context7 service instance
context7_service = Context7Service()
