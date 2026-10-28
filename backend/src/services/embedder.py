"""
Embedding service for generating text embeddings.

Works with OpenAI or any OpenAI-compatible API (e.g. Google Gemini via OPENAI_BASE_URL).
Sends small batches and waits/retries when the API returns a rate-limit (429) error,
so free-tier limits don't make uploads fail.
"""
import time
from typing import List

from openai import OpenAI, RateLimitError
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("embedder")

# Small batches + a short pause keep us under free-tier per-minute limits.
BATCH_SIZE = 20
PAUSE_BETWEEN_BATCHES = 3  # seconds


class EmbedderService:
    """Service for generating embeddings."""

    def __init__(self):
        """Initialize the API client (base URL is read from OPENAI_BASE_URL if set)."""
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.openai_embedding_model

    @retry(
        retry=retry_if_exception_type(RateLimitError),
        stop=stop_after_attempt(6),
        wait=wait_exponential(multiplier=2, min=5, max=65),
        reraise=True,
    )
    def _create(self, inputs):
        """One embeddings API call, retried with waits on rate-limit errors."""
        return self.client.embeddings.create(model=self.model, input=inputs)

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text to embed

        Returns:
            Embedding vector (list of floats)
        """
        try:
            response = self._create(text)
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def embed_batch(self, texts: List[str], batch_size: int = BATCH_SIZE) -> List[List[float]]:
        """
        Generate embeddings for many texts, in small batches.

        Args:
            texts: List of texts to embed
            batch_size: Number of texts per API call (kept small for free tiers)

        Returns:
            List of embedding vectors, in the same order as texts
        """
        batch_size = min(batch_size, BATCH_SIZE)
        all_embeddings: List[List[float]] = []
        total_batches = (len(texts) + batch_size - 1) // batch_size

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_no = i // batch_size + 1

            try:
                response = self._create(batch)
                all_embeddings.extend(data.embedding for data in response.data)
                logger.info(f"Generated {len(batch)} embeddings (batch {batch_no}/{total_batches})")
            except Exception as e:
                logger.error(f"Error generating batch embeddings (batch {batch_no}/{total_batches}): {e}")
                raise

            if batch_no < total_batches:
                time.sleep(PAUSE_BETWEEN_BATCHES)

        return all_embeddings


# Global embedder service instance
embedder_service = EmbedderService()
