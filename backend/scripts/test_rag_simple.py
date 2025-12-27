"""
Simple RAG test script to verify the implementation.

Usage:
    python backend/scripts/test_rag_simple.py
"""
import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.rag_service import rag_service
from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("test_rag")


async def test_rag():
    """Test RAG service with sample questions."""
    logger.info("Starting RAG test...")

    # Connect to database
    await postgres_service.connect()

    try:
        # Test 1: Health check
        logger.info("\n=== Test 1: Health Check ===")
        health = await rag_service.health_check()
        logger.info(f"Health status: {health}")

        # Test 2: Simple question
        logger.info("\n=== Test 2: Simple Question ===")
        question1 = "What is Physical AI?"
        result1 = await rag_service.answer_question(
            question=question1,
            include_sources=True
        )
        logger.info(f"Question: {question1}")
        logger.info(f"Answer: {result1['answer']}")
        logger.info(f"Sources: {len(result1.get('sources', []))} sources found")

        # Test 3: Question with selected text
        logger.info("\n=== Test 3: Question with Selected Text ===")
        question2 = "What does this text explain?"
        selected_text = "Humanoid robots are designed to mimic human form and behavior."
        result2 = await rag_service.answer_question(
            question=question2,
            selected_text=selected_text,
            include_sources=True
        )
        logger.info(f"Question: {question2}")
        logger.info(f"Selected Text: {selected_text}")
        logger.info(f"Answer: {result2['answer']}")

        logger.info("\n=== All Tests Passed! ===")

    except Exception as e:
        logger.error(f"Test failed: {e}")
        raise
    finally:
        await postgres_service.disconnect()


if __name__ == "__main__":
    asyncio.run(test_rag())
