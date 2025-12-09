# Implementation Plan: RAG Chatbot for Physical AI & Humanoid Robotics Book

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

## Summary

Build a production-ready RAG (Retrieval-Augmented Generation) chatbot system for the Physical AI & Humanoid Robotics textbook. The system enables readers to ask questions and receive accurate, cited answers from book content using OpenAI's Agents SDK with automatic tool-calling. Core capabilities include full-book search, selected-text-only mode, and an embeddable Docusaurus chat widget.

**Technical Approach**: FastAPI backend orchestrates a multi-stage RAG pipeline: content chunking (512 tokens/50 overlap) → OpenAI embeddings (text-embedding-3-small) → Qdrant vector storage → semantic search (top_k=20) → reranking (final_k=5) → OpenAI Agents (gpt-4-turbo-preview) with tool-calling for answer generation with citations. Neon Postgres stores metadata and sessions. Better Auth handles authentication.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- Backend: FastAPI 0.109+, OpenAI SDK 1.10+ (Agents/ChatKit), Qdrant Client 1.7+, Psycopg 3.1+ (Neon), Better Auth SDK
- Frontend: Docusaurus 3.x, React 18, TypeScript 5.x
- Processing: tiktoken (tokenization), PyPDF2 or pypdf (PDF parsing)

**Storage**:
- Vector: Qdrant Cloud (collection: textbook_content, 1536 dimensions for text-embedding-3-small)
- Relational: Neon Postgres (chunks metadata, user sessions, conversation history)

**Testing**:
- Backend: pytest, pytest-asyncio, httpx (async client testing)
- Frontend: Jest, React Testing Library
- Integration: End-to-end tests with test fixtures for embeddings/retrieval

**Target Platform**:
- Backend: Linux server (Vercel/Railway deployment)
- Frontend: Static site (GitHub Pages or Vercel)
- Browsers: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+)

**Project Type**: Web application (FastAPI backend + Docusaurus frontend)

**Performance Goals**:
- Query response: < 5 seconds end-to-end (SC-001)
- Ingestion: 300-page book in < 10 minutes (SC-005)
- Concurrency: 100+ concurrent users (SC-006)
- Widget load: < 2 seconds (SC-008)

**Constraints**:
- Retrieval accuracy: 90% for in-book questions (SC-002)
- Citation accuracy: 95% (SC-003)
- Selected-text mode: 100% scoping accuracy (SC-004)
- Uptime: 99% (SC-010)
- Security: 100% authentication enforcement for protected endpoints (SC-012)

**Scale/Scope**:
- Initial: Single 300-page textbook, ~600-800 chunks
- Users: 100+ concurrent, 1000+ total expected
- Conversation history: Last 10 exchanges per session

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Core Principles

✅ **I. Course Alignment & Technical Accuracy**: RAG chatbot enhances learning by providing instant, cited answers from textbook content. Answers must cite exact sources (FR-012), preventing hallucination.

✅ **II. Modular, Maintainable Architecture**: Backend follows FastAPI service structure with clear separation: ingestion service, retrieval service, chat engine, auth service. Frontend widget is embeddable and independent.

✅ **III. Reusable Intelligence via Claude Subagents**: This RAG chatbot itself qualifies as a Claude Subagent for intelligent Q&A. Tool definitions (search_chunks, get_metadata, rerank) are reusable across contexts.

✅ **IV. Functional Completeness**: Implements required RAG chatbot feature with all scoring criteria: full-book mode, selected-text mode, authentication integration, OpenAI Agents tool-calling.

✅ **V. Consistent Code Quality**: All code will be production-ready with error handling, input validation, logging, and comprehensive tests. No hardcoded secrets (use .env).

✅ **VI. Deployment Readiness**: Backend deployable to Vercel/Railway, frontend embeds in Docusaurus site. Deployment configs, environment templates, and build scripts included.

### Quality Gates

- ✅ All API endpoints documented (OpenAPI schema)
- ✅ Error handling comprehensive (API errors, DB failures, OpenAI rate limits)
- ✅ Security: auth tokens, input validation, secrets in environment variables
- ✅ RAG accuracy validated with test questions and ground truth
- ✅ "Answer from selected text" feature functional (FR-008, SC-004)

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0: Technology decisions and best practices
├── data-model.md        # Phase 1: Entity schemas and relationships
├── quickstart.md        # Phase 1: Setup and run instructions
├── contracts/           # Phase 1: API contracts (OpenAPI specs)
│   ├── ingestion.yaml   # POST /ingest endpoint
│   ├── query.yaml       # POST /query endpoint
│   └── retrieval.yaml   # GET /doc/:id endpoint
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist (completed)
└── tasks.md             # Phase 2: Generated by /sp.tasks (not by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py                 # FastAPI app entry point
│   ├── config.py               # Environment config, settings
│   ├── models/
│   │   ├── chunk.py            # Chunk entity, validation
│   │   ├── query.py            # Query request/response models
│   │   └── session.py          # User session models
│   ├── services/
│   │   ├── chunker.py          # Text chunking (512/50 tokens)
│   │   ├── embedder.py         # OpenAI embedding generation
│   │   ├── qdrant_client.py    # Qdrant vector operations
│   │   ├── postgres_client.py  # Neon Postgres metadata ops
│   │   ├── retriever.py        # Search + rerank logic
│   │   └── chat_engine.py      # OpenAI Agents/ChatKit integration
│   ├── tools/
│   │   ├── search_chunks.py    # Tool: semantic search
│   │   ├── get_metadata.py     # Tool: metadata lookup
│   │   └── rerank.py           # Tool: reranking logic
│   ├── api/
│   │   ├── ingestion.py        # POST /ingest endpoint
│   │   ├── query.py            # POST /query endpoint
│   │   ├── retrieval.py        # GET /doc/:id endpoint
│   │   └── auth.py             # Authentication endpoints
│   └── utils/
│       ├── logger.py           # Logging setup
│       └── validators.py       # Input validation helpers
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py
│   │   ├── test_embedder.py
│   │   └── test_retriever.py
│   ├── integration/
│   │   ├── test_ingestion.py   # End-to-end ingestion test
│   │   ├── test_query.py       # End-to-end query test
│   │   └── test_selected_text.py # Selected-text mode test
│   └── fixtures/
│       ├── sample_book.txt     # Test book content
│       └── test_chunks.json    # Pre-generated test chunks
├── requirements.txt            # Python dependencies
├── pyproject.toml             # Project metadata, build config
├── .env.example               # Environment variable template
└── README.md                  # Backend setup instructions

frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.tsx      # Main chat interface component
│   │   ├── MessageList.tsx     # Message display with citations
│   │   ├── InputBox.tsx        # User input handling
│   │   └── CitationLink.tsx    # Clickable citation component
│   ├── hooks/
│   │   ├── useChat.ts          # Chat state management hook
│   │   ├── useTextSelection.ts # Text highlighting logic
│   │   └── useAuth.ts          # Authentication state hook
│   ├── services/
│   │   └── api.ts              # API client for backend endpoints
│   ├── types/
│   │   └── chat.ts             # TypeScript interfaces
│   └── widget.tsx              # Widget entry point (embeddable)
├── tests/
│   ├── ChatWidget.test.tsx
│   ├── useTextSelection.test.ts
│   └── api.test.ts
├── package.json
├── tsconfig.json
└── vite.config.ts             # Build config for widget bundle

deployment/
├── vercel.json                # Vercel deployment config (backend)
├── railway.json               # Railway deployment config (alternative)
└── docs/
    └── deployment.md          # Deployment instructions
```

**Structure Decision**: Web application structure with separate `backend/` and `frontend/` directories. Backend uses FastAPI service-oriented architecture with clear layers: models (entities), services (business logic), tools (OpenAI Agent tools), and api (HTTP endpoints). Frontend is a standalone widget built with React/TypeScript that embeds into Docusaurus.

## Complexity Tracking

No constitutional violations. The architecture follows modular design principles with clear separation of concerns. The multi-service backend (chunker, embedder, retriever, chat engine) is justified by the distinct responsibilities in the RAG pipeline, and each service can be tested independently.

---

## Phase 0: Research & Decisions

**Objective**: Resolve all technical unknowns and document technology choices, best practices, and integration patterns.

### Research Tasks

1. **OpenAI Agents/ChatKit Tool-Calling Pattern**
   - Decision Needed: How to structure tools for automatic invocation
   - Research: OpenAI Agents SDK tool registration, function calling patterns
   - Rationale: Must ensure 100% automatic tool invocation (SC-009)

2. **Qdrant Collection Configuration**
   - Decision Needed: Optimal vector configuration for text-embedding-3-small (1536 dims)
   - Research: Qdrant HNSW parameters, distance metrics (cosine vs dot product)
   - Rationale: Impacts retrieval accuracy (SC-002)

3. **Chunking Strategy for Technical Content**
   - Decision Needed: Validate 512 tokens/50 overlap for robotics textbook
   - Research: Semantic chunking vs fixed-size, chapter boundary handling
   - Rationale: Chunk quality directly impacts retrieval and answer quality

4. **Reranking Algorithm**
   - Decision Needed: Cross-encoder model or simpler scoring approach
   - Research: Cohere rerank, sentence-transformers cross-encoder, or custom scoring
   - Rationale: Improves top-5 precision from top-20 candidates (FR-006)

5. **Selected-Text Mode Implementation**
   - Decision Needed: How frontend captures chunk IDs from highlighted text
   - Research: DOM-based selection, data attributes, chunk-to-DOM mapping
   - Rationale: Critical for selected-text mode feature (FR-008, SC-004)

6. **Better Auth Integration**
   - Decision Needed: Session management, token handling, onboarding flow
   - Research: Better Auth Python SDK, FastAPI middleware patterns
   - Rationale: Required for authentication (FR-016, SC-011, SC-012)

7. **Error Handling Patterns**
   - Decision Needed: Retry logic for OpenAI rate limits, DB connection failures
   - Research: Exponential backoff, circuit breakers, fallback strategies
   - Rationale: Required for 99% uptime (SC-010)

8. **Caching Strategy**
   - Decision Needed: Where to cache (embeddings, frequent queries, metadata)
   - Research: Redis vs in-memory, TTL policies, cache invalidation
   - Rationale: Performance optimization for concurrent users (SC-006)

### Output: research.md

Document format:
```markdown
# Research & Technical Decisions: RAG Chatbot

## Decision: [Topic]
**Chosen**: [Selected approach]
**Rationale**: [Why this choice]
**Alternatives Considered**: [Other options and why rejected]
**Best Practices**: [Key recommendations for implementation]
**References**: [Links to documentation, examples]
```

---

## Phase 1: Architecture & Design

**Prerequisites**: `research.md` completed with all decisions documented

### 1.1 Data Model (`data-model.md`)

Extract entities from spec.md (lines 136-145) and define schemas:

**Entities**:
1. **Book** (Neon Postgres)
   - Fields: id (UUID), title, author, version, total_pages, ingestion_date, status
   - Relationships: 1-to-many with Chunk

2. **Chunk** (Neon Postgres + Qdrant)
   - Postgres Fields: id (UUID), book_id, chunk_index, text, chapter_num, chapter_title, page_num, section_heading, paragraph_index, token_count, created_at
   - Qdrant Fields: vector (1536 dims), payload {chunk_id, book_id, metadata}
   - Validation: token_count must be ≤ 512, chunk_index unique per book

3. **Embedding** (logical entity, stored in Qdrant)
   - Fields: chunk_id (reference), vector, model_version (text-embedding-3-small)
   - Metadata: embedding_date, model_name

4. **Query** (transient, not persisted unless for analytics)
   - Fields: query_text, selected_chunk_ids (optional array), session_id, timestamp
   - Validation: query_text max length 4000 chars, selected_chunk_ids must exist

5. **Answer** (persisted in conversation history)
   - Fields: id (UUID), query_id, answer_text, citations (array), confidence_score, processing_time_ms, created_at
   - Relationships: Many answers per session

6. **Citation** (embedded in Answer)
   - Fields: chunk_id, excerpt (text), chapter_page_ref, relevance_score
   - Validation: excerpt max 200 chars

7. **UserSession** (Neon Postgres)
   - Fields: id (UUID), user_id, auth_token, conversation_history (JSONB), created_at, expires_at, preferences (JSONB)
   - Validation: expires_at > created_at, conversation_history max 10 exchanges

### 1.2 API Contracts (`contracts/` directory)

Generate OpenAPI 3.0 YAML files for each endpoint:

**`contracts/ingestion.yaml`** - POST /ingest
```yaml
paths:
  /ingest:
    post:
      summary: Ingest book content
      requestBody:
        content:
          multipart/form-data:
            schema:
              type: object
              properties:
                file:
                  type: string
                  format: binary
                book_metadata:
                  type: object
                  properties:
                    title: string
                    author: string
                    version: string
      responses:
        '200':
          description: Ingestion started
          content:
            application/json:
              schema:
                type: object
                properties:
                  job_id: string (UUID)
                  status: string (processing)
                  total_chunks: integer
        '400': Bad request (invalid file)
        '500': Internal error
```

**`contracts/query.yaml`** - POST /query
```yaml
paths:
  /query:
    post:
      summary: Ask a question
      requestBody:
        content:
          application/json:
            schema:
              type: object
              required: [query_text]
              properties:
                query_text:
                  type: string
                  maxLength: 4000
                selected_chunk_ids:
                  type: array
                  items:
                    type: string (UUID)
                session_id:
                  type: string (UUID)
      responses:
        '200':
          description: Answer generated
          content:
            application/json:
              schema:
                type: object
                properties:
                  answer_text: string
                  citations:
                    type: array
                    items:
                      type: object
                      properties:
                        chunk_id: string
                        excerpt: string
                        chapter_page_ref: string
                        relevance_score: number
                  confidence_score: number
                  processing_time_ms: integer
        '400': Bad request (invalid query)
        '404': Selected chunks not found
        '429': Rate limit exceeded
        '500': Internal error
```

**`contracts/retrieval.yaml`** - GET /doc/:id
```yaml
paths:
  /doc/{chunk_id}:
    get:
      summary: Retrieve chunk metadata and full text
      parameters:
        - name: chunk_id
          in: path
          required: true
          schema:
            type: string (UUID)
      responses:
        '200':
          description: Chunk data
          content:
            application/json:
              schema:
                type: object
                properties:
                  chunk_id: string
                  text: string
                  chapter_num: integer
                  chapter_title: string
                  page_num: integer
                  section_heading: string
        '404': Chunk not found
        '500': Internal error
```

### 1.3 Quickstart Guide (`quickstart.md`)

**Setup Instructions**:
1. Prerequisites: Python 3.11+, Node 18+, Qdrant Cloud account, Neon Postgres account, OpenAI API key
2. Clone repository and checkout `001-rag-chatbot` branch
3. Backend setup:
   ```bash
   cd backend
   cp .env.example .env
   # Fill in OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, BETTER_AUTH_SECRET
   pip install -r requirements.txt
   uvicorn src.main:app --reload
   ```
4. Frontend setup:
   ```bash
   cd frontend
   npm install
   npm run dev
   ```
5. Test ingestion: `curl -X POST http://localhost:8000/ingest -F "file=@sample_book.pdf"`
6. Test query: `curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d '{"query_text": "What is ROS 2?"}'`

### 1.4 Agent Context Update

Run `.specify/scripts/bash/update-agent-context.sh claude` to add:
- FastAPI, OpenAI Agents SDK, Qdrant Client, Psycopg (Neon)
- RAG pipeline architecture: chunking → embedding → vector search → reranking → LLM generation
- Tool-calling pattern for automatic retrieval

---

## Phase 2: Implementation Phases

**Note**: This section provides high-level phase breakdown. Detailed tasks will be generated by `/sp.tasks` command.

### Phase 2.1: Architecture & Infrastructure (P1)
- Set up FastAPI project structure
- Configure environment variables and secrets management
- Initialize Qdrant collection (textbook_content)
- Set up Neon Postgres schema (books, chunks, sessions tables)
- Implement logging and error handling framework
- **Acceptance**: Health check endpoint returns 200, DB connections verified

### Phase 2.2: Ingestion Pipeline (P1)
- Implement text extraction from PDF/markdown
- Build chunking service (512 tokens, 50 overlap, chapter boundary detection)
- Integrate OpenAI embeddings API (text-embedding-3-small)
- Store vectors in Qdrant, metadata in Neon
- Add ingestion status endpoint
- **Acceptance**: 300-page book ingested in < 10 minutes (SC-005), all chunks stored correctly

### Phase 2.3: RAG Retrieval Pipeline (P1)
- Implement semantic search (query → embedding → Qdrant search, top_k=20)
- Build reranking service (top_20 → final_k=5)
- Create tools for OpenAI Agents:
  - `search_chunks(query, selected_ids)`: semantic search with optional filtering
  - `get_metadata(chunk_id)`: metadata lookup
  - `rerank(chunks)`: relevance scoring
- **Acceptance**: Retrieval accuracy ≥ 90% on test questions (SC-002)

### Phase 2.4: Chat Engine with Tool-Calling (P1)
- Integrate OpenAI Agents SDK (gpt-4-turbo-preview)
- Register tools for automatic invocation
- Implement conversation history tracking (last 10 exchanges)
- Build citation injection logic (inline references to chunks)
- Add confidence scoring
- **Acceptance**: Tool-calling works automatically 100% of time (SC-009), citations 95% accurate (SC-003)

### Phase 2.5: API Endpoints (P1)
- POST /ingest: trigger ingestion pipeline
- POST /query: handle question, invoke chat engine, return answer + citations
- GET /doc/:id: retrieve chunk metadata
- Validate selected-text mode: filter search to selected_chunk_ids
- **Acceptance**: All endpoints functional, selected-text mode 100% accurate (SC-004), response time < 5s (SC-001)

### Phase 2.6: Authentication & Sessions (P3)
- Integrate Better Auth SDK
- Add authentication middleware to protected endpoints
- Implement session management (creation, expiry, token validation)
- Track conversation history per session
- **Acceptance**: Auth completes in < 3s (SC-011), 100% enforcement (SC-012)

### Phase 2.7: Frontend Chat Widget (P2)
- Build ChatWidget React component (message list, input box)
- Implement text highlighting/selection logic
- Capture chunk IDs from selected text (DOM mapping)
- Build API client (query, retrieval endpoints)
- Add typing indicators, loading states
- Implement clickable citations (scroll to source)
- **Acceptance**: Widget loads in < 2s (SC-008), responsive on mobile

### Phase 2.8: Testing & Validation (All Phases)
- Unit tests: chunker, embedder, retriever, each tool
- Integration tests: ingestion end-to-end, query end-to-end, selected-text mode
- Load testing: 100 concurrent users
- Accuracy validation: test question set with ground truth
- **Acceptance**: All tests pass, concurrency target met (SC-006), accuracy targets met (SC-002, SC-003)

### Phase 2.9: Deployment (P2)
- Create `.env.example` template
- Write deployment configs (Vercel, Railway)
- Document environment setup
- Write deployment.md guide
- Test deployment in staging environment
- **Acceptance**: System deployable, uptime ≥ 99% (SC-010)

---

## Key Architectural Decisions (to be documented in research.md)

1. **Tool-Calling Pattern**: OpenAI Agents SDK with function definitions for search_chunks, get_metadata, rerank
2. **Vector Search**: Qdrant with HNSW index, cosine similarity
3. **Reranking**: Cross-encoder model (sentence-transformers) or custom relevance scoring
4. **Chunking**: Fixed 512-token chunks with 50-token overlap, respect chapter boundaries where possible
5. **Citation Format**: Inline references with chunk_id, chapter/page, clickable links
6. **Session Management**: JWT tokens from Better Auth, JSONB conversation history in Postgres
7. **Error Handling**: Exponential backoff for OpenAI rate limits, circuit breaker for DB failures
8. **Caching**: In-memory cache for frequently asked questions (optional optimization)

---

## Testing Strategy

### Unit Tests
- Chunker: token counts, overlap validation, chapter boundary handling
- Embedder: API integration, error handling, batch processing
- Retriever: semantic search accuracy, filtering by selected_chunk_ids
- Tools: search_chunks, get_metadata, rerank functions

### Integration Tests
- Ingestion: Upload book → verify chunks in Qdrant + Neon
- Query: Submit question → verify answer with citations
- Selected-text mode: Query with chunk IDs → verify restricted search

### End-to-End Tests
- Full RAG pipeline: Ingest → query → answer with citations
- Conversation flow: Multi-turn questions with context
- Widget integration: UI interactions, citation clicks

### Performance Tests
- Concurrency: 100+ simultaneous users
- Response time: < 5s for 95th percentile
- Ingestion speed: 300 pages in < 10 minutes

---

## Deployment Plan

### Environment Variables (`.env.example`)
```bash
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
QDRANT_COLLECTION=textbook_content

# Neon Postgres
DATABASE_URL=postgresql://user:pass@host/db

# Better Auth
BETTER_AUTH_SECRET=...
BETTER_AUTH_URL=...

# App Config
ENVIRONMENT=production
LOG_LEVEL=INFO
MAX_CHUNK_SIZE=512
CHUNK_OVERLAP=50
TOP_K_CANDIDATES=20
FINAL_K_RESULTS=5
```

### Deployment Steps
1. **Backend (Vercel or Railway)**:
   - Configure environment variables
   - Set build command: `pip install -r requirements.txt`
   - Set start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - Deploy from `001-rag-chatbot` branch

2. **Frontend Widget**:
   - Build widget bundle: `npm run build`
   - Deploy to CDN or embed directly in Docusaurus
   - Configure API endpoint URL

3. **Database Setup**:
   - Run Neon Postgres schema migrations
   - Create Qdrant collection with 1536-dim vectors

4. **Smoke Tests**:
   - Health check endpoint
   - Test ingestion with sample content
   - Test query with known questions
   - Verify authentication flow

---

## Risks & Mitigations (from spec.md)

### Risk 1: Answer Quality and Hallucination
**Mitigation**: Strict citation requirements (FR-012), confidence scoring, explicit "not in book" responses

### Risk 2: Retrieval Accuracy
**Mitigation**: Tune chunk size/overlap, query expansion, reranking (FR-006), monitor metrics (SC-002)

### Risk 3: Performance and Scalability
**Mitigation**: Caching, connection pooling, async processing, rate limiting (SC-006)

### Risk 4: API Cost and Rate Limits
**Mitigation**: Cache embeddings, batch requests, usage quotas, monitor costs

### Risk 5: Data Privacy and Security
**Mitigation**: Encryption at rest/transit, access controls, audit logs, secure env var management

---

## Next Steps

1. **Complete Phase 0**: Run research tasks, document decisions in `research.md`
2. **Complete Phase 1**: Generate `data-model.md`, `contracts/`, `quickstart.md`, update agent context
3. **Generate Tasks**: Run `/sp.tasks` to create detailed, testable tasks organized by user story
4. **Implementation**: Execute tasks using `/sp.implement`
5. **ADR Review**: Identify architecturally significant decisions and create ADRs using `/sp.adr`

---

**Plan Status**: ✅ Complete | **Ready for**: `/sp.tasks` command
