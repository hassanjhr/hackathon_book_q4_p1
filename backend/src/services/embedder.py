"""
OpenAI embedding service for generating text embeddings.
"""
from typing import List
from openai import OpenAI
from tenacity import retry, stop_after_attempt, wait_exponential
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("embedder")


class EmbedderService:
    """Service for generating embeddings using OpenAI API."""

    def __init__(self):
        """Initialize OpenAI client."""
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.openai_embedding_model

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10)
    )
    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text to embed

        Returns:
            Embedding vector (list of floats)
        """
        try:
            response = self.client.embeddings.create(
                model=self.model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10)
    )
    def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of texts to embed
            batch_size: Number of texts to embed in each API call

        Returns:
            List of embedding vectors
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            try:
                response = self.client.embeddings.create(
                    model=self.model,
                    input=batch
                )
                embeddings = [data.embedding for data in response.data]
                all_embeddings.extend(embeddings)

                logger.info(f"Generated {len(embeddings)} embeddings (batch {i // batch_size + 1})")
            except Exception as e:
                logger.error(f"Error generating batch embeddings: {e}")
                raise

        return all_embeddings


# Global embedder service instance
embedder_service = EmbedderService()
