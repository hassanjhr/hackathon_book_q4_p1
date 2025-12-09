"""
Ingestion API endpoints for uploading and processing book content.
"""
from fastapi import APIRouter, File, UploadFile, Form, HTTPException
from src.models.ingestion import IngestionResponse
from src.services.ingestion_service import ingestion_service
from src.utils.logger import get_logger

logger = get_logger("ingestion_api")

router = APIRouter()


@router.post("/ingest", response_model=IngestionResponse)
async def ingest_book(
    file: UploadFile = File(...),
    title: str = Form(...),
    author: str = Form(None),
    version: str = Form(None)
):
    """
    Ingest a book file for RAG processing.

    Args:
        file: Book file upload (text, PDF, etc.)
        title: Book title
        author: Book author (optional)
        version: Book version (optional)

    Returns:
        IngestionResponse with job_id and status
    """
    try:
        # Read file content
        content = await file.read()

        # Decode text (assuming UTF-8)
        try:
            text = content.decode('utf-8')
        except UnicodeDecodeError:
            # Try latin-1 as fallback
            text = content.decode('latin-1')

        logger.info(f"Received file: {file.filename}, size: {len(text)} characters")

        # Process ingestion
        result = await ingestion_service.ingest_text(
            text=text,
            title=title,
            author=author,
            version=version
        )

        return IngestionResponse(
            job_id=result["job_id"],
            status=result["status"],
            total_chunks=result.get("total_chunks"),
            message=result["message"]
        )

    except Exception as e:
        logger.error(f"Error during ingestion: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/ingest/status/{job_id}")
async def get_ingestion_status(job_id: str):
    """
    Get the status of an ingestion job.

    Args:
        job_id: UUID of the ingestion job (book ID)

    Returns:
        Job status information
    """
    try:
        # Query book status from database
        from src.services.postgres_client import postgres_service

        book = await postgres_service.fetch_one(
            "SELECT id, title, status, ingestion_date FROM books WHERE id = $1",
            job_id
        )

        if not book:
            raise HTTPException(status_code=404, detail="Job not found")

        # Get chunk count
        chunk_count = await postgres_service.fetch_one(
            "SELECT COUNT(*) as count FROM chunks WHERE book_id = $1",
            job_id
        )

        return {
            "job_id": str(book["id"]),
            "title": book["title"],
            "status": book["status"],
            "total_chunks": chunk_count["count"] if chunk_count else 0,
            "ingestion_date": book["ingestion_date"]
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching job status: {e}")
        raise HTTPException(status_code=500, detail=str(e))
