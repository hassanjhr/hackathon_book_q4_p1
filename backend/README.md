---
title: RAG Chatbot Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
app_port: 7860
---

# AI-Native Textbook RAG Chatbot Backend

FastAPI backend for Physical AI & Humanoid Robotics interactive textbook with RAG chatbot.

## Features

- RAG (Retrieval-Augmented Generation) with Qdrant vector database
- Context7 integration for live library documentation
- OpenAI GPT-4 responses with source citations
- Debug endpoint: `/api/rag/debug`

## Environment Variables Required

Set these in Hugging Face Spaces Settings → Secrets:

```bash
QDRANT_URL=your-cluster.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_content
OPENAI_API_KEY=sk-your-openai-key
```

**IMPORTANT**:
- ❌ Do NOT include `https://` in QDRANT_URL
- ✅ DO include port `:6333`

## API Endpoints

- `GET /health` - Health check
- `GET /api/rag/debug` - Diagnostic information
- `POST /api/rag/chat` - Chat endpoint

## Deployment

This Space uses Docker. The Dockerfile is configured to run on port 7860.

## Troubleshooting

Visit `/api/rag/debug` to diagnose connection issues.
