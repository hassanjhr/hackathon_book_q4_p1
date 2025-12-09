"""
Logging configuration for structured logging.
"""
import logging
import sys
from typing import Optional

# Create logger
logger = logging.getLogger("rag_chatbot")
logger.setLevel(logging.INFO)

# Create console handler with formatting
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)

# Create formatter
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
handler.setFormatter(formatter)

# Add handler to logger
logger.addHandler(handler)


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """
    Get a logger instance.

    Args:
        name: Optional name for the logger. If None, returns root logger.

    Returns:
        Logger instance
    """
    if name:
        return logging.getLogger(f"rag_chatbot.{name}")
    return logger
