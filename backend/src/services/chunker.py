"""
Text chunking service with token-based chunking using tiktoken.
"""
import tiktoken
from typing import List, Dict, Any
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("chunker")


class ChunkerService:
    """Service for chunking text into token-based segments."""

    def __init__(self):
        """Initialize chunker with tiktoken encoding."""
        self.encoding = tiktoken.get_encoding("cl100k_base")  # GPT-4 encoding
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap

    def chunk_text(self, text: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Chunk text into overlapping token-based segments.

        Args:
            text: Input text to chunk
            metadata: Optional metadata to attach to chunks

        Returns:
            List of chunk dictionaries with text and metadata
        """
        if not text or not text.strip():
            return []

        # Encode text to tokens
        tokens = self.encoding.encode(text)
        total_tokens = len(tokens)

        chunks = []
        chunk_index = 0
        start_idx = 0

        while start_idx < total_tokens:
            # Calculate end index for this chunk
            end_idx = min(start_idx + self.chunk_size, total_tokens)

            # Get chunk tokens
            chunk_tokens = tokens[start_idx:end_idx]

            # Decode back to text
            chunk_text = self.encoding.decode(chunk_tokens)

            # Create chunk dictionary
            chunk = {
                "chunk_index": chunk_index,
                "text": chunk_text,
                "token_count": len(chunk_tokens),
                "start_token": start_idx,
                "end_token": end_idx
            }

            # Add metadata if provided
            if metadata:
                chunk.update(metadata)

            chunks.append(chunk)

            # Move to next chunk with overlap
            chunk_index += 1
            start_idx = end_idx - self.chunk_overlap

            # Break if we're at the end
            if end_idx >= total_tokens:
                break

        logger.info(f"Created {len(chunks)} chunks from {total_tokens} tokens")
        return chunks

    def detect_chapter_boundaries(self, text: str) -> List[Dict[str, Any]]:
        """
        Detect chapter boundaries in text (simple regex-based approach).

        Args:
            text: Input text

        Returns:
            List of chapter boundaries with positions
        """
        import re

        # Simple pattern for chapter detection
        chapter_pattern = r'(Chapter\s+\d+|CHAPTER\s+\d+|Ch\.\s*\d+)'

        chapters = []
        for match in re.finditer(chapter_pattern, text):
            chapters.append({
                "position": match.start(),
                "text": match.group(0),
                "chapter_num": int(re.search(r'\d+', match.group(0)).group())
            })

        return chapters


# Global chunker service instance
chunker_service = ChunkerService()
