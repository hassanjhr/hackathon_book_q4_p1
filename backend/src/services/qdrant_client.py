"""
Qdrant vector database client for storing and searching embeddings.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
)
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("qdrant_client")


class QdrantService:
    """Service for interacting with Qdrant vector database."""

    def __init__(self):
        """Initialize Qdrant client and create collection if needed."""

        # 🔥 IMPORTANT FIX: force REST, disable gRPC, enable HTTPS for Qdrant Cloud
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,   # Use REST API instead of gRPC
            https=True,          # Enable HTTPS for Qdrant Cloud
            timeout=30
        )

        self.collection_name = settings.qdrant_collection_name
        self.is_available = False  # Track if Qdrant is accessible

        # Try to ensure collection exists, but don't fail if Qdrant is unavailable
        try:
            self._ensure_collection_exists()
            self.is_available = True
            logger.info("✅ Qdrant connection successful")
        except Exception as e:
            logger.warning(f"⚠️ Qdrant unavailable (continuing without vector search): {str(e)[:100]}")
            logger.warning("The backend will start, but vector search features will be disabled")

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
        try:
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

            # Try new API first (qdrant-client >= 1.8), fallback to old API
            try:
                # New API (>= 1.8.0)
                results = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_vector,
                    limit=limit,
                    score_threshold=score_threshold,
                    query_filter=query_filter
                ).points
            except AttributeError:
                # Old API (<= 1.7.0)
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