"""
Qdrant vector database client for storing and searching embeddings.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("qdrant_client")


class QdrantService:
    """Service for interacting with Qdrant vector database."""

    def __init__(self):
        """Initialize Qdrant client and create collection if needed."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection_name
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Create collection if it doesn't exist."""
        try:
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                logger.info(f"Creating collection: {self.collection_name}")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=settings.embedding_dimensions,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Collection {self.collection_name} created successfully")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
            raise

    def insert_vectors(
        self,
        chunk_ids: List[str],
        vectors: List[List[float]],
        payloads: List[Dict[str, Any]]
    ) -> bool:
        """
        Insert vectors with metadata into Qdrant.

        Args:
            chunk_ids: List of chunk IDs
            vectors: List of embedding vectors
            payloads: List of metadata dictionaries

        Returns:
            True if successful, False otherwise
        """
        try:
            points = [
                PointStruct(
                    id=chunk_id,
                    vector=vector,
                    payload=payload
                )
                for chunk_id, vector, payload in zip(chunk_ids, vectors, payloads)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Inserted {len(points)} vectors into Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error inserting vectors: {e}")
            return False

    def search_vectors(
        self,
        query_vector: List[float],
        limit: int = 20,
        score_threshold: Optional[float] = None,
        chunk_ids_filter: Optional[List[str]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant.

        Args:
            query_vector: Query embedding vector
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score threshold
            chunk_ids_filter: Optional list of chunk IDs to filter search

        Returns:
            List of search results with chunk_id, score, and payload
        """
        try:
            # Build filter if chunk_ids are provided
            query_filter = None
            if chunk_ids_filter:
                query_filter = Filter(
                    must=[
                        FieldCondition(
                            key="chunk_id",
                            match=MatchValue(value=cid)
                        )
                        for cid in chunk_ids_filter
                    ]
                )

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
                query_filter=query_filter
            )

            return [
                {
                    "chunk_id": str(result.id),
                    "score": result.score,
                    "payload": result.payload
                }
                for result in results
            ]
        except Exception as e:
            logger.error(f"Error searching vectors: {e}")
            return []

    def delete_vectors(self, chunk_ids: List[str]) -> bool:
        """
        Delete vectors by chunk IDs.

        Args:
            chunk_ids: List of chunk IDs to delete

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=chunk_ids
            )
            logger.info(f"Deleted {len(chunk_ids)} vectors from Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error deleting vectors: {e}")
            return False


# Global Qdrant service instance
qdrant_service = QdrantService()
