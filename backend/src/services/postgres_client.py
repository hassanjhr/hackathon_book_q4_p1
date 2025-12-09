"""
Neon Postgres database client for storing metadata.
"""
import asyncpg
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager
from src.config import settings
from src.utils.logger import get_logger

logger = get_logger("postgres_client")


class PostgresService:
    """Service for interacting with Neon Postgres database."""

    def __init__(self):
        """Initialize Postgres service."""
        self.pool: Optional[asyncpg.Pool] = None
        self.database_url = settings.database_url

    async def connect(self):
        """Create connection pool to Postgres database."""
        try:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=2,
                max_size=10,
                command_timeout=60
            )
            logger.info("Connected to Postgres database")
        except Exception as e:
            logger.error(f"Error connecting to Postgres: {e}")
            raise

    async def disconnect(self):
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Disconnected from Postgres database")

    @asynccontextmanager
    async def get_connection(self):
        """Get a database connection from the pool."""
        if not self.pool:
            await self.connect()

        async with self.pool.acquire() as connection:
            yield connection

    async def execute(self, query: str, *args) -> str:
        """
        Execute a query that doesn't return rows.

        Args:
            query: SQL query to execute
            *args: Query parameters

        Returns:
            Query execution status
        """
        async with self.get_connection() as conn:
            try:
                result = await conn.execute(query, *args)
                return result
            except Exception as e:
                logger.error(f"Error executing query: {e}")
                raise

    async def fetch_one(self, query: str, *args) -> Optional[Dict[str, Any]]:
        """
        Fetch a single row from a query.

        Args:
            query: SQL query to execute
            *args: Query parameters

        Returns:
            Dictionary of column: value pairs, or None if no row found
        """
        async with self.get_connection() as conn:
            try:
                row = await conn.fetchrow(query, *args)
                return dict(row) if row else None
            except Exception as e:
                logger.error(f"Error fetching row: {e}")
                raise

    async def fetch_all(self, query: str, *args) -> List[Dict[str, Any]]:
        """
        Fetch all rows from a query.

        Args:
            query: SQL query to execute
            *args: Query parameters

        Returns:
            List of dictionaries (column: value pairs)
        """
        async with self.get_connection() as conn:
            try:
                rows = await conn.fetch(query, *args)
                return [dict(row) for row in rows]
            except Exception as e:
                logger.error(f"Error fetching rows: {e}")
                raise

    async def insert_book(
        self,
        title: str,
        author: Optional[str] = None,
        version: Optional[str] = None,
        total_pages: Optional[int] = None
    ) -> str:
        """
        Insert a new book record.

        Args:
            title: Book title
            author: Book author
            version: Book version
            total_pages: Total number of pages

        Returns:
            Book ID (UUID as string)
        """
        query = """
            INSERT INTO books (title, author, version, total_pages, status)
            VALUES ($1, $2, $3, $4, 'processing')
            RETURNING id
        """
        result = await self.fetch_one(query, title, author, version, total_pages)
        return str(result['id']) if result else None

    async def insert_chunks(self, chunks: List[Dict[str, Any]]) -> int:
        """
        Insert multiple chunk records.

        Args:
            chunks: List of chunk dictionaries

        Returns:
            Number of chunks inserted
        """
        query = """
            INSERT INTO chunks (
                id, book_id, chunk_index, text, chapter_num, chapter_title,
                page_num, section_heading, paragraph_index, token_count
            )
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
        """
        async with self.get_connection() as conn:
            try:
                await conn.executemany(
                    query,
                    [
                        (
                            chunk['id'],
                            chunk['book_id'],
                            chunk['chunk_index'],
                            chunk['text'],
                            chunk.get('chapter_num'),
                            chunk.get('chapter_title'),
                            chunk.get('page_num'),
                            chunk.get('section_heading'),
                            chunk.get('paragraph_index'),
                            chunk['token_count']
                        )
                        for chunk in chunks
                    ]
                )
                logger.info(f"Inserted {len(chunks)} chunks into Postgres")
                return len(chunks)
            except Exception as e:
                logger.error(f"Error inserting chunks: {e}")
                raise

    async def get_chunk(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Get chunk metadata by ID.

        Args:
            chunk_id: Chunk UUID

        Returns:
            Chunk metadata dictionary or None if not found
        """
        query = """
            SELECT id, book_id, chunk_index, text, chapter_num, chapter_title,
                   page_num, section_heading, paragraph_index, token_count, created_at
            FROM chunks
            WHERE id = $1
        """
        return await self.fetch_one(query, chunk_id)

    async def update_book_status(self, book_id: str, status: str) -> bool:
        """
        Update book ingestion status.

        Args:
            book_id: Book UUID
            status: New status (e.g., 'completed', 'failed')

        Returns:
            True if successful
        """
        query = "UPDATE books SET status = $1 WHERE id = $2"
        await self.execute(query, status, book_id)
        return True


# Global Postgres service instance
postgres_service = PostgresService()
