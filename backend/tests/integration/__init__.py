"""
Integration tests for RAG chatbot.

These tests verify end-to-end functionality with actual services.
They are skipped by default and require:
- Qdrant service running
- Neon Postgres running
- OpenAI API key configured

Run with: pytest tests/integration/ -m integration
"""
