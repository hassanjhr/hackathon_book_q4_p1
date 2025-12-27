"""
Database setup script - creates tables if they don't exist.

Usage:
    python backend/scripts/setup_db.py
"""
import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.postgres_client import postgres_service
from src.utils.logger import get_logger

logger = get_logger("setup_db")


async def setup_database():
    """Create database tables if they don't exist."""
    logger.info("Setting up database...")

    # Read schema file
    schema_file = Path(__file__).parent.parent / "migrations" / "001_initial_schema.sql"

    if not schema_file.exists():
        logger.error(f"Schema file not found: {schema_file}")
        return False

    with open(schema_file, 'r') as f:
        schema_sql = f.read()

    # Connect to database
    await postgres_service.connect()

    try:
        # Execute schema SQL
        await postgres_service.execute(schema_sql)
        logger.info("Database schema created successfully")
        return True
    except Exception as e:
        logger.error(f"Error creating database schema: {e}")
        return False
    finally:
        await postgres_service.disconnect()


if __name__ == "__main__":
    success = asyncio.run(setup_database())
    if success:
        logger.info("Database setup complete!")
    else:
        logger.error("Database setup failed!")
        sys.exit(1)
