# RAG Chatbot for Physical AI & Humanoid Robotics Book

> **Update (Oct 2026):** Migrated from OpenAI to Google Gemini (via the OpenAI-compatible API), moved to a new Qdrant Cloud cluster, and added rate-limit handling for embeddings. Models and provider are configured through environment variables (`OPENAI_BASE_URL`, `OPENAI_CHAT_MODEL`, `OPENAI_EMBEDDING_MODEL`, `EMBEDDING_DIMENSIONS`).

A RAG (Retrieval-Augmented Generation) chatbot system that enables readers to ask questions and receive cited answers from the Physical AI & Humanoid Robotics textbook.

## 🔗 Live Demo

| What | Link |
|---|---|
| 📖 **Textbook website** (with chatbot) | [hackathon-book-q4-p1.vercel.app](https://hackathon-book-q4-p1.vercel.app) |
| 🤗 **Backend (Hugging Face Space)** | [huggingface.co/spaces/hassanjhr/rag_chatbot](https://huggingface.co/spaces/hassanjhr/rag_chatbot) |
| 📑 **API documentation (Swagger)** | [hassanjhr-rag-chatbot.hf.space/docs](https://hassanjhr-rag-chatbot.hf.space/docs) |

> The backend runs on a free Hugging Face Space. If it has been idle, the first question may take 1–2 minutes while the Space wakes up.

## Features

- **Content Ingestion**: Upload book content and automatically chunk, embed, and store for retrieval
- **Full-Book Search**: Ask questions across the entire book content
- **Selected-Text Mode**: Ask questions scoped to specific highlighted text
- **Tool-Calling Agent**: Automatic tool-calling for retrieval, metadata lookup, and answer generation
- **Citations**: Answers include references to the textbook sources used
- **Vector Search**: Semantic search using Qdrant (3072-dim with Gemini, 1536-dim with OpenAI)
- **Metadata Storage**: Structured metadata in Neon Postgres
- **Rate-Limit Handling**: Small embedding batches with automatic retry, so free-tier API limits don't break ingestion

## Tech Stack

- **Backend**: FastAPI (Python 3.11+), deployed on Hugging Face Spaces
- **AI/ML**: OpenAI SDK with Google Gemini (`gemini-embedding-001`, `gemini-3.5-flash-lite`); also works with OpenAI models
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres (async)
- **Frontend**: Docusaurus + React/TypeScript chat widget, deployed on Vercel

## Prerequisites

- Python 3.11 or higher
- Node.js 18 or higher
- Qdrant Cloud account
- Neon Postgres database
- Google Gemini API key (free from [Google AI Studio](https://aistudio.google.com)) or an OpenAI API key

## Quick Start

### 1. Clone and Setup

```bash
git clone https://github.com/hassanjhr/hackathon_book_q4_p1.git
cd hackathon_book_q4_p1
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Environment Configuration

Create a `.env` file in the `backend/` directory.

**Option A: Google Gemini (free tier)**

```bash
OPENAI_API_KEY=your_gemini_api_key_here
OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
OPENAI_CHAT_MODEL=gemini-3.5-flash-lite
OPENAI_EMBEDDING_MODEL=gemini-embedding-001
EMBEDDING_DIMENSIONS=3072

QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=textbook_content_gemini

DATABASE_URL=postgresql://user:password@host/database
```

**Option B: OpenAI**

```bash
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_CHAT_MODEL=gpt-4o-mini
# Defaults: text-embedding-3-small, 1536 dimensions

QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=textbook_content

DATABASE_URL=postgresql://user:password@host/database
```

Optional:

```bash
BETTER_AUTH_SECRET=your_secret_key_here
CLAUDE_API_KEY=your_claude_api_key_here
```

**Important**: Never commit `.env` files to version control!

> Embedding sizes differ between providers. When switching providers, use a new `QDRANT_COLLECTION_NAME` and re-ingest the book.

### 4. Database Setup

Run the database schema migration:

```bash
# Connect to your Neon database and run:
psql $DATABASE_URL -f migrations/001_initial_schema.sql
```

### 5. Run the Backend

```bash
# From the backend/ directory
uvicorn src.main:app --reload
```

The API will be available at `http://localhost:8000`

API Documentation: `http://localhost:8000/docs`

### 6. Frontend Setup (Docusaurus)

```bash
# From the repository root
npm --prefix my-website install
npm --prefix my-website run start
```

The Docusaurus site will be available at `http://localhost:3000`

## Usage

### 1. Ingest Book Content

Option 1: index all chapters from `my-website/docs/` with the script:

```bash
python backend/scripts/index_content.py
```

Option 2: upload a file through the API (also available in the Swagger UI at `/docs`):

```bash
curl -X POST "http://localhost:8000/api/ingest" \
  -F "file=@/path/to/your/book.md" \
  -F "title=Physical AI & Humanoid Robotics" \
  -F "author=Your Name" \
  -F "version=1.0"
```

Response:
```json
{
  "job_id": "123e4567-e89b-12d3-a456-426614174000",
  "status": "completed",
  "total_chunks": 150,
  "message": "Book ingested successfully"
}
```

### 2. Check Ingestion Status

```bash
curl "http://localhost:8000/api/ingest/status/{job_id}"
```

### 3. Ask Questions (Chatbot)

```bash
curl -X POST "http://localhost:8000/api/rag/chat" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

### 4. Ask Questions (Query API)

```bash
curl -X POST "http://localhost:8000/api/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What are the main components of a humanoid robot?"
  }'
```

### 5. Get Specific Chunk

```bash
curl "http://localhost:8000/api/doc/{chunk_id}"
```

### 6. Diagnose Qdrant Connection

```bash
curl "http://localhost:8000/api/rag/debug"
```

## Testing

### Run Tests

```bash
cd backend
pytest tests/
```

### Manual Smoke Tests

1. **Health Check**:
   ```bash
   curl http://localhost:8000/health
   ```

2. **Ingest Test Book**:
   ```bash
   echo "Chapter 1: Introduction to Humanoid Robotics..." > test_book.txt

   curl -X POST "http://localhost:8000/api/ingest" \
     -F "file=@test_book.txt" \
     -F "title=Test Book"
   ```

3. **Query Test**:
   ```bash
   curl -X POST "http://localhost:8000/api/query" \
     -H "Content-Type: application/json" \
     -d '{"query_text": "What is this book about?"}'
   ```

## Project Structure

```
├── backend/
│   ├── src/
│   │   ├── api/           # API endpoints (ingestion, query, rag, chatkit)
│   │   ├── models/        # Pydantic models
│   │   ├── services/      # Business logic
│   │   │   ├── chunker.py
│   │   │   ├── embedder.py
│   │   │   ├── qdrant_client.py
│   │   │   ├── postgres_client.py
│   │   │   ├── ingestion_service.py
│   │   │   ├── rag_service.py
│   │   │   └── chat_engine.py
│   │   ├── tools/         # Agent tools
│   │   ├── utils/         # Utilities
│   │   ├── config.py      # Configuration
│   │   └── main.py        # FastAPI app
│   ├── scripts/           # Indexing and setup scripts
│   ├── migrations/        # Database schemas
│   ├── tests/             # Test files
│   └── requirements.txt   # Python dependencies
├── my-website/            # Docusaurus frontend (textbook + chat widget)
├── specs/                 # Feature specifications
└── README.md              # This file
```

## Architecture

### RAG Pipeline

1. **Ingestion**: `Upload → Chunk (512 tokens / 50 overlap) → Embed → Store (Qdrant + Neon)`
2. **Retrieval**: `Query → Embed → Search (top_k=20) → Select top results (final_k=5)`
3. **Generation**: `LLM → Answer + Sources`

### Tool-Calling

The system uses function calling to automatically invoke tools:

- **search_chunks**: Semantic search in Qdrant
- **get_metadata**: Retrieve full chunk metadata from Postgres

## Security

- **Environment Variables**: All secrets in `.env` locally and in Hugging Face Space secrets in production (never committed)
- **Input Validation**: Query length limits, chunk ID validation
- **CORS**: Configured for the deployed frontend

## Troubleshooting

### Database Connection Errors

```bash
psql $DATABASE_URL -c "SELECT 1"
```

### Qdrant Connection Errors

```bash
curl -H "api-key: $QDRANT_API_KEY" "$QDRANT_URL/collections"
```

Or open `/api/rag/debug` on the running backend.

### Rate-Limit (429) Errors

The embedder retries automatically with increasing waits. For large books, ingest the content in smaller parts.

### Model Not Found (404) Errors

Model names change over time. Update `OPENAI_CHAT_MODEL` / `OPENAI_EMBEDDING_MODEL` to a currently available model.

## Next Steps

- [x] Add frontend chat widget to Docusaurus
- [x] Deploy to production (Vercel + Hugging Face Spaces)
- [x] Support Google Gemini through the OpenAI-compatible API
- [ ] Implement authentication with Better Auth
- [ ] Add conversation history persistence
- [ ] Add monitoring and analytics

## Contributing

See `specs/001-rag-chatbot/` for detailed specifications and task breakdown.

## Author

**Muhammad Hassan Jawaid**: [GitHub](https://github.com/hassanjhr) · [LinkedIn](https://www.linkedin.com/in/muhammad-hassan-jawaid-580796123)
