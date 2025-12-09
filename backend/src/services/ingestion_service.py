"""
Ingestion service that orchestrates the chunking → embedding → storage pipeline.
"""
from typing import Optional
from uuid import uuid4
from src.services.chunker import chunker_service
from src.services.embedder import embedder_service
from src.services.qdrant_client import qdrant_service
from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("ingestion_service")


class IngestionService:
    """Service for coordinating book content ingestion."""

    async def ingest_text(
        self,
        text: str,
        title: str,
        author: Optional[str] = None,
        version: Optional[str] = None
    ) -> dict:
        """
        Ingest book text through the complete RAG pipeline.

        Args:
            text: Full book text content
            title: Book title
            author: Book author
            version: Book version

        Returns:
            Dictionary with job_id, status, and total_chunks
        """
        try:
            # Step 1: Create book record in database
            book_id = await postgres_service.insert_book(
                title=title,
                author=author,
                version=version
            )
            logger.info(f"Created book record: {book_id}")

            # Step 2: Chunk the text
            chunks = chunker_service.chunk_text(text)
            logger.info(f"Created {len(chunks)} chunks")

            if not chunks:
                await postgres_service.update_book_status(book_id, "failed")
                return {
                    "job_id": book_id,
                    "status": "failed",
                    "message": "No chunks created from text"
                }

            # Step 3: Generate embeddings
            chunk_texts = [chunk["text"] for chunk in chunks]
            embeddings = embedder_service.embed_batch(chunk_texts)
            logger.info(f"Generated {len(embeddings)} embeddings")

            # Step 4: Prepare chunk data with IDs
            chunk_records = []
            chunk_ids = []
            vectors = []
            payloads = []

            for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                chunk_id = str(uuid4())
                chunk_ids.append(chunk_id)
                vectors.append(embedding)

                # Metadata for Qdrant
                payloads.append({
                    "chunk_id": chunk_id,
                    "book_id": str(book_id),
                    "chunk_index": chunk["chunk_index"],
                    "text": chunk["text"][:500],  # Store abbreviated text in payload
                    "token_count": chunk["token_count"]
                })

                # Full record for Postgres
                chunk_records.append({
                    "id": chunk_id,
                    "book_id": str(book_id),
                    "chunk_index": chunk["chunk_index"],
                    "text": chunk["text"],
                    "chapter_num": chunk.get("chapter_num"),
                    "chapter_title": chunk.get("chapter_title"),
                    "page_num": chunk.get("page_num"),
                    "section_heading": chunk.get("section_heading"),
                    "paragraph_index": chunk.get("paragraph_index"),
                    "token_count": chunk["token_count"]
                })

            # Step 5: Store in Qdrant (vectors)
            qdrant_success = qdrant_service.insert_vectors(
                chunk_ids=chunk_ids,
                vectors=vectors,
                payloads=payloads
            )

            if not qdrant_success:
                await postgres_service.update_book_status(book_id, "failed")
                return {
                    "job_id": book_id,
                    "status": "failed",
                    "message": "Failed to store vectors in Qdrant"
                }

            # Step 6: Store in Postgres (metadata)
            await postgres_service.insert_chunks(chunk_records)

            # Step 7: Update book status
            await postgres_service.update_book_status(book_id, "completed")

            logger.info(f"Ingestion completed successfully for book {book_id}")

            return {
                "job_id": book_id,
                "status": "completed",
                "total_chunks": len(chunks),
                "message": "Book ingested successfully"
            }

        except Exception as e:
            logger.error(f"Error during ingestion: {e}")
            if 'book_id' in locals():
                await postgres_service.update_book_status(book_id, "failed")
            raise


# Global ingestion service instance
ingestion_service = IngestionService()
