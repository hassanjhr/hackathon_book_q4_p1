"""
Content indexing script for RAG chatbot.

Recursively scans markdown files from my-website/docs/, chunks text,
generates embeddings, and stores them in Qdrant and Postgres.

Usage:
    python backend/scripts/index_content.py
"""
import asyncio
import os
import re
import sys
from pathlib import Path
from typing import List, Dict, Any
from uuid import uuid4

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.embedder import embedder_service
from src.services.chunker import chunker_service
from src.services.qdrant_client import qdrant_service
from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("index_content")


def extract_frontmatter_and_content(text: str) -> tuple[Dict[str, str], str]:
    """
    Extract YAML frontmatter and clean content from markdown.

    Args:
        text: Raw markdown text

    Returns:
        Tuple of (frontmatter_dict, clean_content)
    """
    frontmatter = {}
    content = text

    # Check for frontmatter (starts with ---)
    if text.startswith('---'):
        parts = text.split('---', 2)
        if len(parts) >= 3:
            # Parse frontmatter (simple key-value extraction)
            fm_text = parts[1]
            for line in fm_text.strip().split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip().strip('"\'')
            content = parts[2].strip()

    return frontmatter, content


def clean_markdown_content(content: str) -> str:
    """
    Clean markdown content for better indexing.

    Args:
        content: Raw markdown content

    Returns:
        Cleaned text
    """
    # Remove code fences but keep the code (optional: could skip code blocks)
    content = re.sub(r'```[\w]*\n(.*?)\n```', r'\1', content, flags=re.DOTALL)

    # Remove HTML comments
    content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

    # Remove excessive whitespace
    content = re.sub(r'\n\s*\n\s*\n', '\n\n', content)

    # Keep markdown headers as they provide context
    # Keep links, lists, etc. as they contain valuable information

    return content.strip()


def extract_module_and_week(file_path: str) -> tuple[str, str]:
    """
    Extract module and week from file path.

    Args:
        file_path: Path to markdown file

    Returns:
        Tuple of (module, week)
    """
    path_parts = Path(file_path).parts

    module = "Unknown"
    week = "Unknown"

    for part in path_parts:
        if part.startswith('module-'):
            module = f"Module {part.split('-')[1]}"
        if part.startswith('week'):
            # Extract week number from filename like "week1-foundations.md"
            match = re.match(r'week(\d+)', part)
            if match:
                week = f"Week {match.group(1)}"

    return module, week


def scan_markdown_files(docs_dir: str) -> List[Dict[str, str]]:
    """
    Recursively scan for markdown files.

    Args:
        docs_dir: Root documentation directory

    Returns:
        List of file info dictionaries
    """
    markdown_files = []
    docs_path = Path(docs_dir)

    if not docs_path.exists():
        logger.error(f"Documentation directory not found: {docs_dir}")
        return []

    # Recursively find all .md files
    for md_file in docs_path.rglob("*.md"):
        if md_file.is_file():
            markdown_files.append({
                'path': str(md_file),
                'relative_path': str(md_file.relative_to(docs_path.parent))
            })

    logger.info(f"Found {len(markdown_files)} markdown files")
    return markdown_files


async def index_documents():
    """Main indexing function."""
    logger.info("Starting content indexing...")

    # Connect to Postgres
    await postgres_service.connect()

    # Get project root
    project_root = Path(__file__).parent.parent.parent
    docs_dir = project_root / "my-website" / "docs"

    logger.info(f"Scanning documentation directory: {docs_dir}")

    # Scan for markdown files
    markdown_files = scan_markdown_files(str(docs_dir))

    if not markdown_files:
        logger.error("No markdown files found!")
        return

    # Create a book entry in Postgres
    book_id = await postgres_service.insert_book(
        title="Physical AI & Humanoid Robotics Textbook",
        author="AI-Native Learning",
        version="1.0"
    )
    logger.info(f"Created book record with ID: {book_id}")

    # Process each markdown file
    total_chunks = 0
    global_chunk_index = 0  # Global counter across all files
    all_chunk_records = []

    for file_info in markdown_files:
        file_path = file_info['path']
        logger.info(f"Processing: {file_path}")

        try:
            # Read file
            with open(file_path, 'r', encoding='utf-8') as f:
                raw_content = f.read()

            # Extract frontmatter and content
            frontmatter, content = extract_frontmatter_and_content(raw_content)

            # Clean content
            clean_content = clean_markdown_content(content)

            if not clean_content.strip():
                logger.warning(f"Empty content after cleaning: {file_path}")
                continue

            # Extract module and week
            module, week = extract_module_and_week(file_path)

            # Get title from frontmatter or filename
            title = frontmatter.get('title', frontmatter.get('sidebar_label', Path(file_path).stem))

            # Chunk the content
            chunks = chunker_service.chunk_text(
                text=clean_content,
                metadata={
                    'module': module,
                    'week': week,
                    'file_path': file_info['relative_path'],
                    'title': title
                }
            )

            if not chunks:
                logger.warning(f"No chunks created for: {file_path}")
                continue

            logger.info(f"Created {len(chunks)} chunks from {file_path}")

            # Prepare chunk records for Postgres
            chunk_records = []
            for chunk in chunks:
                chunk_id = str(uuid4())
                chunk_record = {
                    'id': chunk_id,
                    'book_id': book_id,
                    'chunk_index': global_chunk_index,  # Use global counter
                    'text': chunk['text'],
                    'section_heading': chunk.get('title'),
                    'token_count': chunk['token_count']
                }
                chunk_records.append(chunk_record)

                # Store chunk with metadata for vector insertion
                chunk['chunk_id'] = chunk_id
                chunk['book_id'] = book_id

                global_chunk_index += 1  # Increment global counter

            # Insert chunks into Postgres
            await postgres_service.insert_chunks(chunk_records)
            all_chunk_records.extend(chunks)
            total_chunks += len(chunks)

            logger.info(f"Inserted {len(chunks)} chunks into Postgres")

        except Exception as e:
            logger.error(f"Error processing {file_path}: {e}")
            continue

    logger.info(f"Total chunks created: {total_chunks}")

    # Generate embeddings in batches
    logger.info("Generating embeddings...")

    texts = [chunk['text'] for chunk in all_chunk_records]
    embeddings = embedder_service.embed_batch(texts, batch_size=100)

    logger.info(f"Generated {len(embeddings)} embeddings")

    # Prepare for Qdrant insertion
    chunk_ids = [chunk['chunk_id'] for chunk in all_chunk_records]
    payloads = [
        {
            'chunk_id': chunk['chunk_id'],
            'book_id': chunk['book_id'],
            'module': chunk.get('module', 'Unknown'),
            'week': chunk.get('week', 'Unknown'),
            'file_path': chunk.get('file_path', ''),
            'title': chunk.get('title', ''),
            'chunk_index': chunk['chunk_index'],
            'text': chunk['text'][:500]  # Store first 500 chars in payload
        }
        for chunk in all_chunk_records
    ]

    # Insert into Qdrant
    logger.info("Inserting vectors into Qdrant...")
    success = qdrant_service.insert_vectors(chunk_ids, embeddings, payloads)

    if success:
        logger.info("Successfully inserted all vectors into Qdrant")

        # Update book status to completed
        await postgres_service.update_book_status(book_id, 'completed')
        logger.info(f"Book indexing completed. Total chunks: {total_chunks}")
    else:
        logger.error("Failed to insert vectors into Qdrant")
        await postgres_service.update_book_status(book_id, 'failed')

    # Disconnect from Postgres
    await postgres_service.disconnect()

    logger.info("Indexing complete!")


if __name__ == "__main__":
    asyncio.run(index_documents())
