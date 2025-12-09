# RAG Chatbot for Physical AI & Humanoid Robotics Book

A production-ready RAG (Retrieval-Augmented Generation) chatbot system that enables readers to ask questions and receive accurate, cited answers from the Physical AI & Humanoid Robotics textbook.

## Features

- **Content Ingestion**: Upload book content and automatically chunk, embed, and store for retrieval
- **Full-Book Search**: Ask questions across the entire book content
- **Selected-Text Mode**: Ask questions scoped to specific highlighted text
- **OpenAI Agents**: Automatic tool-calling for retrieval, metadata lookup, and answer generation
- **Citations**: Every answer includes citations with chapter and page references
- **Vector Search**: Semantic search using Qdrant with 1536-dimensional embeddings
- **Metadata Storage**: Structured metadata in Neon Postgres

## Tech Stack

- **Backend**: FastAPI (Python 3.11+)
- **AI/ML**: OpenAI SDK (text-embedding-3-small, gpt-4-turbo-preview)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres (async)
- **Frontend**: Docusaurus + React/TypeScript widget

## Prerequisites

- Python 3.11 or higher
- Node.js 18 or higher
- Qdrant Cloud account
- Neon Postgres database
- OpenAI API key

## Quick Start

### 1. Clone and Setup

```bash
git clone <repository-url>
cd hackathon_book_q4_p1
git checkout 001-rag-chatbot
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

Create a `.env` file in the `backend/` directory:

```bash
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# Neon Postgres Configuration
DATABASE_URL=postgresql://user:password@host/database

# Optional: Better Auth (for authentication)
BETTER_AUTH_SECRET=your_secret_key_here

# Optional: Claude API (for additional features)
CLAUDE_API_KEY=your_claude_api_key_here
```

**Important**: Never commit `.env` files to version control!

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

First, upload a book to the system:

```bash
curl -X POST "http://localhost:8000/api/ingest" \
  -F "file=@/path/to/your/book.txt" \
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

### 3. Ask Questions (Full-Book Mode)

```bash
curl -X POST "http://localhost:8000/api/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What are the main components of a humanoid robot?"
  }'
```

Response:
```json
{
  "answer_text": "The main components of a humanoid robot include...",
  "citations": [
    {
      "chunk_id": "abc-123",
      "excerpt": "Humanoid robots typically consist of...",
      "chapter_page_ref": "Chapter 2, Page 15",
      "relevance_score": 0.95
    }
  ],
  "processing_time_ms": 2300,
  "mode": "full-book"
}
```

### 4. Ask Questions (Selected-Text Mode)

```bash
curl -X POST "http://localhost:8000/api/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "Explain this in simpler terms",
    "selected_chunk_ids": ["chunk-id-1", "chunk-id-2"]
  }'
```

### 5. Get Specific Chunk

```bash
curl "http://localhost:8000/api/doc/{chunk_id}"
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
   # Create a test file
   echo "Chapter 1: Introduction to Humanoid Robotics..." > test_book.txt

   # Ingest it
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
│   │   ├── api/           # API endpoints
│   │   │   ├── ingestion.py
│   │   │   └── query.py
│   │   ├── models/        # Pydantic models
│   │   │   ├── book.py
│   │   │   ├── chunk.py
│   │   │   ├── ingestion.py
│   │   │   └── query.py
│   │   ├── services/      # Business logic
│   │   │   ├── chunker.py
│   │   │   ├── embedder.py
│   │   │   ├── qdrant_client.py
│   │   │   ├── postgres_client.py
│   │   │   ├── ingestion_service.py
│   │   │   └── chat_engine.py
│   │   ├── tools/         # OpenAI Agent tools
│   │   │   ├── search_chunks.py
│   │   │   └── get_metadata.py
│   │   ├── utils/         # Utilities
│   │   │   ├── logger.py
│   │   │   └── validators.py
│   │   ├── config.py      # Configuration
│   │   └── main.py        # FastAPI app
│   ├── migrations/        # Database schemas
│   ├── tests/             # Test files
│   └── requirements.txt   # Python dependencies
├── my-website/            # Docusaurus frontend
├── specs/                 # Feature specifications
└── README.md              # This file
```

## Architecture

### RAG Pipeline

1. **Ingestion**: `Upload → Chunk (512 tokens/50 overlap) → Embed → Store (Qdrant + Neon)`
2. **Retrieval**: `Query → Embed → Search (top_k=20) → Rerank (final_k=5)`
3. **Generation**: `OpenAI Agent → Tool Calls → Answer + Citations`

### OpenAI Agents & Tool-Calling

The system uses OpenAI's function calling to automatically invoke tools:

- **search_chunks**: Semantic search in Qdrant
- **get_metadata**: Retrieve full chunk metadata from Postgres

The agent autonomously decides when to call these tools based on the user's question.

## Performance

- Query response: < 5 seconds
- Ingestion: 300-page book in < 10 minutes
- Retrieval accuracy: 90%+
- Citation accuracy: 95%+

## Security

- **Environment Variables**: All secrets in `.env` (not committed)
- **Input Validation**: Query length limits, chunk ID validation
- **Sanitization**: SQL injection prevention, XSS protection
- **CORS**: Configured for development (update for production)

## Troubleshooting

### Database Connection Errors

```bash
# Verify DATABASE_URL is correct
echo $DATABASE_URL

# Test connection
psql $DATABASE_URL -c "SELECT 1"
```

### Qdrant Connection Errors

```bash
# Verify Qdrant credentials
curl -H "api-key: $QDRANT_API_KEY" "$QDRANT_URL/collections"
```

### OpenAI API Errors

```bash
# Verify API key
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

## Next Steps

- [ ] Add frontend chat widget to Docusaurus
- [ ] Implement authentication with Better Auth
- [ ] Add conversation history persistence
- [ ] Deploy to production (Vercel/Railway)
- [ ] Add monitoring and analytics

## Contributing

See `specs/001-rag-chatbot/` for detailed specifications and task breakdown.

## License

[Your License Here]
