"""
Test script for Context7 integration with RAG chatbot.

Usage:
    python backend/scripts/test_context7.py
"""
import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.rag_service import rag_service
from src.services.context7_service import context7_service
from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("test_context7")


async def test_library_detection():
    """Test library detection functionality."""
    logger.info("\n=== Test 1: Library Detection ===")

    test_questions = [
        ("How do I create a PyTorch tensor?", "pytorch", True),
        ("What is reinforcement learning?", None, False),
        ("How do I create a ROS2 node?", "ros2", True),
        ("Explain NVIDIA Isaac Sim", "isaac-sim", True),
    ]

    for question, expected_lib, expected_is_lib in test_questions:
        result = await rag_service._detect_library_intent(question)

        logger.info(f"\nQuestion: {question}")
        logger.info(f"Result: {result}")

        is_correct = (
            result["is_library_question"] == expected_is_lib
            and (not expected_lib or result.get("library_name") == expected_lib)
        )

        logger.info(f"✓ PASS" if is_correct else f"✗ FAIL")


async def test_context7_service():
    """Test Context7 service directly."""
    logger.info("\n=== Test 2: Context7 Service ===")

    # Test library resolution
    logger.info("\n2a. Testing library resolution...")
    libraries = ["pytorch", "torch", "ros2", "isaac-sim"]

    for lib in libraries:
        lib_id = context7_service.resolve_library(lib)
        logger.info(f"{lib} → {lib_id}")

    # Test documentation fetch (with mock data)
    logger.info("\n2b. Testing documentation fetch...")
    result = await context7_service.search_library_docs(
        library_name="pytorch",
        query="How to create a tensor?",
        mode="code"
    )

    if result:
        logger.info("✓ Successfully fetched documentation")
        logger.info(f"Content length: {len(result.get('content', ''))}")
        logger.info(f"Sources: {result.get('sources', [])}")
    else:
        logger.info("✗ Failed to fetch documentation")

    # Test supported libraries
    logger.info("\n2c. Supported libraries:")
    supported = context7_service.get_supported_libraries()
    logger.info(f"{supported}")


async def test_hybrid_rag():
    """Test hybrid RAG with Context7."""
    logger.info("\n=== Test 3: Hybrid RAG ===")

    # Connect to database
    await postgres_service.connect()

    try:
        # Test 1: Library question with Context7 enabled
        logger.info("\n3a. Library question WITH Context7...")
        result1 = await rag_service.answer_question(
            question="How do I create a PyTorch tensor?",
            use_library_docs=True,
            include_sources=True
        )

        logger.info(f"Answer: {result1['answer'][:200]}...")
        logger.info(f"Number of sources: {len(result1.get('sources', []))}")

        has_library_source = any(
            s.get('type') == 'library_docs'
            for s in result1.get('sources', [])
        )
        logger.info(f"Has library docs: {has_library_source}")
        logger.info("✓ PASS" if has_library_source else "✗ FAIL")

        # Test 2: Same question WITHOUT Context7
        logger.info("\n3b. Same question WITHOUT Context7...")
        result2 = await rag_service.answer_question(
            question="How do I create a PyTorch tensor?",
            use_library_docs=False,
            include_sources=True
        )

        logger.info(f"Answer: {result2['answer'][:200]}...")
        logger.info(f"Number of sources: {len(result2.get('sources', []))}")

        has_only_textbook = all(
            s.get('type') == 'textbook'
            for s in result2.get('sources', [])
        )
        logger.info(f"Has only textbook: {has_only_textbook}")
        logger.info("✓ PASS" if has_only_textbook else "✗ FAIL")

        # Test 3: Non-library question
        logger.info("\n3c. Non-library question...")
        result3 = await rag_service.answer_question(
            question="What is reinforcement learning?",
            use_library_docs=True,
            include_sources=True
        )

        logger.info(f"Answer: {result3['answer'][:200]}...")

        has_no_library = all(
            s.get('type') != 'library_docs'
            for s in result3.get('sources', [])
        )
        logger.info(f"No library docs (expected): {has_no_library}")
        logger.info("✓ PASS" if has_no_library else "✗ FAIL")

    finally:
        await postgres_service.disconnect()


async def run_all_tests():
    """Run all Context7 integration tests."""
    logger.info("=" * 60)
    logger.info("Context7 Integration Tests")
    logger.info("=" * 60)

    try:
        await test_library_detection()
        await test_context7_service()
        await test_hybrid_rag()

        logger.info("\n" + "=" * 60)
        logger.info("All Tests Complete!")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"\nTest suite failed: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(run_all_tests())
