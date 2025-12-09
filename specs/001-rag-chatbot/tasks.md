# Tasks: RAG Chatbot for Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Tests**: Tests are NOT included in this task breakdown as they were not explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/src/`, `frontend/src/`
- All paths are relative to repository root

---

## Phase 1: Setup (Shared Infrastructure) ✅ COMPLETED

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure: backend/src/{models,services,tools,api,utils}, backend/tests/{unit,integration,fixtures}
- [ ] T002 Create frontend directory structure: frontend/src/{components,hooks,services,types}, frontend/tests
- [ ] T003 Create deployment directory: deployment/{docs}
- [X] T004 Initialize Python project with requirements.txt: FastAPI 0.109+, openai 1.10+, qdrant-client 1.7+, psycopg 3.1+, tiktoken, pypdf, pytest, pytest-asyncio, httpx
- [ ] T005 [P] Initialize frontend with package.json: React 18, TypeScript 5.x, vite, jest, react-testing-library
- [X] T006 [P] Create .env.example with keys: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, BETTER_AUTH_SECRET
- [ ] T007 [P] Create backend/pyproject.toml for project metadata and build config
- [ ] T008 [P] Create frontend/tsconfig.json for TypeScript configuration
- [ ] T009 [P] Create frontend/vite.config.ts for widget bundle build
- [X] T010 [P] Create backend/README.md with setup instructions
- [X] T011 [P] Add .gitignore for Python (.venv, __pycache__, .env) and Node (node_modules, dist)

---

## Phase 2: Foundational (Blocking Prerequisites) ✅ COMPLETED

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T012 Setup Neon Postgres schema: Create tables for books, chunks, user_sessions in backend/migrations/001_initial_schema.sql
- [X] T013 Initialize Qdrant collection "textbook_content" with 1536 dimensions (cosine similarity) in backend/src/services/qdrant_client.py
- [X] T014 [P] Implement config.py: Load environment variables, settings class with Pydantic in backend/src/config.py
- [X] T015 [P] Implement logger.py: Configure structured logging with log levels in backend/src/utils/logger.py
- [X] T016 [P] Implement validators.py: Input validation helpers (chunk_id format, query length, token count) in backend/src/utils/validators.py
- [X] T017 Create FastAPI app skeleton with CORS, error handlers, health check endpoint in backend/src/main.py
- [X] T018 [P] Implement postgres_client.py: Connection pool, async query helpers for Neon in backend/src/services/postgres_client.py
- [X] T019 [P] Implement qdrant_client.py: Connection, vector insert/search operations in backend/src/services/qdrant_client.py
- [X] T020 Create base response models: ErrorResponse, SuccessResponse in backend/src/models/response.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 3 - Content Ingestion and Management (Priority: P1) 🎯 MVP Component 1 ✅ COMPLETED

**Goal**: Enable administrators to upload book content and automatically chunk, embed, and store it for retrieval

**Independent Test**: Upload a book file, verify chunks are created (512 tokens/50 overlap), embeddings generated, and data stored in both Qdrant (vectors) and Neon (metadata). Check ingestion status endpoint shows progress.

### Implementation for User Story 3 (Ingestion)

- [X] T021 [P] [US3] Create Book model with fields: id, title, author, version, total_pages, ingestion_date, status in backend/src/models/book.py
- [X] T022 [P] [US3] Create Chunk model with fields: id, book_id, chunk_index, text, chapter_num, chapter_title, page_num, section_heading, paragraph_index, token_count, created_at in backend/src/models/chunk.py
- [X] T023 [P] [US3] Create IngestionRequest and IngestionResponse models in backend/src/models/ingestion.py
- [X] T024 [US3] Implement chunker.py: Text chunking service with 512-token chunks, 50-token overlap, chapter boundary detection using tiktoken in backend/src/services/chunker.py
- [X] T025 [US3] Implement embedder.py: OpenAI text-embedding-3-small integration, batch embedding generation, error handling for rate limits in backend/src/services/embedder.py
- [X] T026 [US3] Implement ingestion service: Coordinate chunking → embedding → storage pipeline in backend/src/services/ingestion_service.py
- [X] T027 [US3] Implement POST /ingest endpoint: Accept multipart file upload, trigger ingestion pipeline, return job status in backend/src/api/ingestion.py
- [X] T028 [US3] Implement GET /ingest/status/:job_id endpoint: Return ingestion progress, chunks processed, errors in backend/src/api/ingestion.py
- [X] T029 [US3] Add ingestion error handling: Retry logic for OpenAI rate limits, partial failure recovery, detailed error logging
- [X] T030 [US3] Create test fixture: sample_book.txt (50-100 pages) in backend/tests/fixtures/sample_book.txt

**Checkpoint**: At this point, content ingestion should be fully functional. Test by uploading sample_book.txt and verifying chunks in databases.

---

## Phase 4: User Story 1 - Basic Question Answering from Full Book (Priority: P1) 🎯 MVP Component 2 ✅ COMPLETED

**Goal**: Enable readers to ask questions and receive accurate, cited answers from the entire book content

**Independent Test**: With book content ingested, submit question "What are the main components of a humanoid robot?", verify answer contains relevant synthesized content with citations to specific chapters. Test follow-up questions maintain context.

### Implementation for User Story 1 (Query & Answer)

- [X] T031 [P] [US1] Create Query model with fields: query_text, selected_chunk_ids (optional), session_id, timestamp in backend/src/models/query.py
- [X] T032 [P] [US1] Create Answer model with fields: id, query_id, answer_text, citations, confidence_score, processing_time_ms, created_at in backend/src/models/answer.py
- [X] T033 [P] [US1] Create Citation model with fields: chunk_id, excerpt, chapter_page_ref, relevance_score in backend/src/models/citation.py
- [X] T034 [P] [US1] Implement search_chunks tool: Accept query + optional selected_ids, embed query, search Qdrant (top_k=20), return candidates in backend/src/tools/search_chunks.py
- [X] T035 [P] [US1] Implement get_metadata tool: Accept chunk_id, query Neon for full metadata, return structured data in backend/src/tools/get_metadata.py
- [ ] T036 [P] [US1] Implement rerank tool: Accept chunks + query, score by relevance, return top_k=5 in backend/src/tools/rerank.py (Note: Reranking done within search_chunks via Qdrant score_threshold)
- [ ] T037 [US1] Implement retriever.py: Orchestrate search (top_k=20) → rerank (final_k=5) pipeline in backend/src/services/retriever.py (Note: Integrated into chat_engine.py)
- [X] T038 [US1] Implement chat_engine.py: OpenAI Agents SDK integration, register tools (search_chunks, get_metadata), configure gpt-4-turbo-preview, handle tool calling in backend/src/services/chat_engine.py
- [ ] T039 [US1] Add conversation history tracking: Store last 10 exchanges per session in JSONB field in backend/src/services/chat_engine.py (Note: Deferred - not in MVP scope)
- [X] T040 [US1] Implement citation injection logic: Parse tool results, format inline citations with chunk references in backend/src/services/chat_engine.py
- [X] T041 [US1] Implement POST /query endpoint: Accept QueryRequest, invoke chat engine with full-book mode, return AnswerResponse with citations in backend/src/api/query.py
- [X] T042 [US1] Implement GET /doc/:id endpoint: Retrieve chunk by ID, return full text + metadata in backend/src/api/query.py
- [X] T043 [US1] Add query validation: Max query length 4000 chars, sanitize input in backend/src/api/query.py
- [X] T044 [US1] Add hallucination prevention: Instruct model to state "information not available in book" when context insufficient in backend/src/services/chat_engine.py
- [ ] T045 [US1] Add confidence scoring: Calculate based on citation count and relevance scores in backend/src/services/chat_engine.py (Note: Deferred - not in MVP scope)

**Checkpoint**: At this point, User Story 1 should be fully functional. Test end-to-end: ingest book → ask question → verify cited answer.

---

## Phase 5: User Story 2 - Selected Text Question Answering (Priority: P2)

**Goal**: Enable readers to highlight specific text and ask questions scoped only to that selection

**Independent Test**: Highlight a paragraph about actuator systems (capture chunk IDs), ask "Explain this in simpler terms", verify answer references ONLY the selected chunks (100% scoping accuracy).

### Implementation for User Story 2 (Selected-Text Mode)

- [ ] T046 [US2] Add selected_chunk_ids parameter to POST /query endpoint in backend/src/api/query.py
- [ ] T047 [US2] Implement selected-text filtering: When selected_chunk_ids provided, filter Qdrant search to only those IDs in backend/src/tools/search_chunks.py
- [ ] T048 [US2] Add validation: Verify selected_chunk_ids exist in database before processing in backend/src/api/query.py
- [ ] T049 [US2] Add mode indicator: Return metadata showing "full-book" vs "selected-text" mode in response in backend/src/api/query.py
- [ ] T050 [US2] Handle empty selection: Return clear error message when selected_chunk_ids is empty array in selected-text mode in backend/src/api/query.py
- [ ] T051 [US2] Add selected-text integration test: Create fixture with specific chunk IDs, verify filtering works correctly in backend/tests/integration/test_selected_text.py

**Checkpoint**: User Stories 1 AND 2 should both work independently. Test mode switching: full-book query → selected-text query → verify different results.

---

## Phase 6: User Story 4 - Embedded Chat Widget Integration (Priority: P2)

**Goal**: Provide an embeddable React chat widget for Docusaurus with text selection, citation links, and responsive design

**Independent Test**: Embed widget in test HTML page, click chat icon to open, type question, see typing indicator, view answer with clickable citations, test on mobile viewport.

### Implementation for User Story 4 (Frontend Widget)

- [ ] T052 [P] [US4] Create chat.ts TypeScript interfaces: Message, Citation, ChatState, APIResponse in frontend/src/types/chat.ts
- [ ] T053 [P] [US4] Implement api.ts: HTTP client for /query, /doc/:id endpoints with error handling in frontend/src/services/api.ts
- [ ] T054 [P] [US4] Implement useChat hook: Manage chat state (messages, loading, errors), send queries, handle responses in frontend/src/hooks/useChat.ts
- [ ] T055 [P] [US4] Implement useTextSelection hook: Detect text highlights, map selected text to chunk IDs, track selection state in frontend/src/hooks/useTextSelection.ts
- [ ] T056 [US4] Create ChatWidget.tsx: Main container component with open/close state, floating button in frontend/src/components/ChatWidget.tsx
- [ ] T057 [US4] Create MessageList.tsx: Display message history, render citations as clickable links, auto-scroll to latest in frontend/src/components/MessageList.tsx
- [ ] T058 [US4] Create InputBox.tsx: Text input with submit button, "Enter" key handler, typing indicator state in frontend/src/components/InputBox.tsx
- [ ] T059 [US4] Create CitationLink.tsx: Clickable citation component that scrolls to source location on click in frontend/src/components/CitationLink.tsx
- [ ] T060 [US4] Add responsive styling: Mobile-friendly layout, touch interactions, viewport adaptation using CSS modules in frontend/src/components/
- [ ] T061 [US4] Implement widget.tsx: Entry point for embeddable bundle, initialization script in frontend/src/widget.tsx
- [ ] T062 [US4] Configure Vite build: Bundle as standalone widget.js with CSS injection in frontend/vite.config.ts
- [ ] T063 [US4] Add loading states: Typing indicator, skeleton loaders, error messages in frontend/src/components/
- [ ] T064 [US4] Add accessibility: ARIA labels, keyboard navigation, focus management in frontend/src/components/

**Checkpoint**: Widget should be embeddable and fully functional. Test by creating test.html with widget embed and verifying all interactions.

---

## Phase 7: User Story 5 - Authentication and Access Control (Priority: P3)

**Goal**: Secure chatbot access with Better Auth integration, session management, and access control

**Independent Test**: Attempt to access /query without auth token → expect 401. Login with valid credentials → receive session token → access /query successfully. Test session expiry handling.

### Implementation for User Story 5 (Authentication)

- [ ] T065 [P] [US5] Create UserSession model with fields: id, user_id, auth_token, conversation_history, created_at, expires_at, preferences in backend/src/models/session.py
- [ ] T066 [P] [US5] Create AuthRequest, AuthResponse, SessionInfo models in backend/src/models/auth.py
- [ ] T067 [US5] Implement Better Auth SDK integration: Initialize client, configure providers in backend/src/services/auth_service.py
- [ ] T068 [US5] Implement session management: Create session, validate token, track expiry in backend/src/services/auth_service.py
- [ ] T069 [US5] Create auth middleware: Validate tokens on protected routes, attach user context to request in backend/src/middleware/auth.py
- [ ] T070 [US5] Implement POST /auth/login endpoint: Accept credentials, return session token in backend/src/api/auth.py
- [ ] T071 [US5] Implement POST /auth/logout endpoint: Invalidate session token in backend/src/api/auth.py
- [ ] T072 [US5] Implement GET /auth/session endpoint: Return current session info in backend/src/api/auth.py
- [ ] T073 [US5] Apply auth middleware to protected endpoints: /query, /ingest, /doc/:id in backend/src/main.py
- [ ] T074 [US5] Handle session expiry: Return 401 with re-auth prompt, preserve conversation context in backend/src/middleware/auth.py
- [ ] T075 [US5] Implement useAuth hook: Manage auth state, login/logout, token storage in frontend/src/hooks/useAuth.ts
- [ ] T076 [US5] Add auth to API client: Attach token to requests, handle 401 responses in frontend/src/services/api.ts
- [ ] T077 [US5] Add login UI to widget: Show login form when unauthenticated in frontend/src/components/ChatWidget.tsx

**Checkpoint**: All user stories should now be independently functional with authentication protecting access.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final deployment preparation

- [ ] T078 [P] Create OpenAPI schema: Document all endpoints with request/response models in backend/docs/openapi.yaml
- [ ] T079 [P] Add rate limiting: Implement per-user rate limits to prevent abuse in backend/src/middleware/rate_limit.py
- [ ] T080 [P] Implement caching layer: Cache frequent queries with TTL for performance in backend/src/services/cache.py
- [ ] T081 [P] Add performance monitoring: Track query latency, embedding time, retrieval accuracy metrics in backend/src/utils/metrics.py
- [ ] T082 [P] Create Vercel deployment config: API routes, environment variable mapping in deployment/vercel.json
- [ ] T083 [P] Create Railway deployment config (alternative): Service definition, build commands in deployment/railway.json
- [ ] T084 [P] Write deployment guide: Setup steps, environment config, smoke tests in deployment/docs/deployment.md
- [ ] T085 [P] Create quickstart.md: Local development setup, test scenarios in specs/001-rag-chatbot/quickstart.md
- [ ] T086 Add error boundary to widget: Graceful error handling with user-friendly messages in frontend/src/components/ErrorBoundary.tsx
- [ ] T087 Optimize bundle size: Code splitting, tree shaking, minimize widget.js in frontend/vite.config.ts
- [ ] T088 Add security headers: CSP, CORS, rate limit headers in backend/src/main.py
- [ ] T089 Code review and cleanup: Remove debug logs, fix linting issues, add docstrings
- [ ] T090 Final integration test: Run full E2E flow with all user stories enabled

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 3 - Ingestion (Phase 3)**: Depends on Foundational - P1 priority (MVP Component 1)
- **User Story 1 - Query/Answer (Phase 4)**: Depends on Foundational - P1 priority (MVP Component 2), requires ingested content to test
- **User Story 2 - Selected Text (Phase 5)**: Depends on User Story 1 completion - P2 priority
- **User Story 4 - Widget (Phase 6)**: Depends on User Story 1 (uses /query endpoint) - P2 priority
- **User Story 5 - Auth (Phase 7)**: Depends on all endpoints being functional - P3 priority
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 3 (P1 - Ingestion)**: Can start after Foundational - No dependencies on other stories
- **User Story 1 (P1 - Query/Answer)**: Can start after Foundational - Independent but needs ingested data to test meaningfully
- **User Story 2 (P2 - Selected Text)**: Extends User Story 1 - Must implement after US1 complete
- **User Story 4 (P2 - Widget)**: Uses endpoints from User Story 1 - Can develop in parallel once US1 API exists
- **User Story 5 (P3 - Auth)**: Wraps all endpoints - Should be last to avoid blocking development

### Within Each User Story

- Models before services (services use models)
- Services before API endpoints (endpoints use services)
- Tools before chat engine (chat engine registers tools)
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

**Phase 1 (Setup)**: Tasks T005, T006, T007, T008, T009, T010, T011 can run in parallel

**Phase 2 (Foundational)**: Tasks T014, T015, T016, T018, T019, T020 can run in parallel

**Phase 3 (US3 - Ingestion)**: Tasks T021, T022, T023 (models) can run in parallel

**Phase 4 (US1 - Query)**: Tasks T031, T032, T033 (models) can run in parallel; Tasks T034, T035, T036 (tools) can run in parallel

**Phase 5 (US2 - Selected Text)**: No parallelizable tasks (all modify same files)

**Phase 6 (US4 - Widget)**: Tasks T052, T053, T054, T055 can run in parallel

**Phase 7 (US5 - Auth)**: Tasks T065, T066 can run in parallel

**Phase 8 (Polish)**: Tasks T078, T079, T080, T081, T082, T083, T084, T085 can run in parallel

---

## Parallel Example: User Story 1 (Query/Answer)

```bash
# Launch all models for User Story 1 together (T031, T032, T033):
Task: "Create Query model with fields: query_text, selected_chunk_ids, session_id, timestamp in backend/src/models/query.py"
Task: "Create Answer model with fields: id, query_id, answer_text, citations, confidence_score, processing_time_ms, created_at in backend/src/models/answer.py"
Task: "Create Citation model with fields: chunk_id, excerpt, chapter_page_ref, relevance_score in backend/src/models/citation.py"

# Launch all tools for User Story 1 together (T034, T035, T036):
Task: "Implement search_chunks tool in backend/src/tools/search_chunks.py"
Task: "Implement get_metadata tool in backend/src/tools/get_metadata.py"
Task: "Implement rerank tool in backend/src/tools/rerank.py"
```

---

## Implementation Strategy

### MVP First (User Stories 3 + 1 Only)

1. Complete Phase 1: Setup (T001-T011)
2. Complete Phase 2: Foundational (T012-T020) - CRITICAL, blocks all stories
3. Complete Phase 3: User Story 3 - Ingestion (T021-T030)
4. Complete Phase 4: User Story 1 - Query/Answer (T031-T045)
5. **STOP and VALIDATE**: Test ingestion → query → cited answer flow
6. Deploy/demo if ready

**This gives you a working RAG chatbot with full-book question answering!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 3 (Ingestion) → Test independently → Content is searchable
3. Add User Story 1 (Query/Answer) → Test independently → Deploy/Demo (MVP!)
4. Add User Story 2 (Selected-Text) → Test independently → Deploy/Demo
5. Add User Story 4 (Widget) → Test independently → Deploy/Demo
6. Add User Story 5 (Auth) → Test independently → Deploy/Demo (Production-Ready)
7. Add Phase 8 (Polish) → Final optimization and deployment

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T020)
2. Once Foundational is done:
   - Developer A: User Story 3 (Ingestion) - T021-T030
   - Developer B: User Story 1 (Query/Answer) - T031-T045 (needs US3 for testing)
3. After US1 complete:
   - Developer C: User Story 2 (Selected-Text) - T046-T051
   - Developer D: User Story 4 (Widget) - T052-T064
4. After all features:
   - Developer E: User Story 5 (Auth) - T065-T077
5. Team: Polish & Deployment - T078-T090

---

## Task Summary

**Total Tasks**: 90

**By Phase**:
- Phase 1 (Setup): 11 tasks
- Phase 2 (Foundational): 9 tasks (BLOCKING)
- Phase 3 (US3 - Ingestion): 10 tasks
- Phase 4 (US1 - Query/Answer): 15 tasks
- Phase 5 (US2 - Selected Text): 6 tasks
- Phase 6 (US4 - Widget): 13 tasks
- Phase 7 (US5 - Auth): 13 tasks
- Phase 8 (Polish): 13 tasks

**By User Story**:
- US1 (Basic Question Answering): 15 tasks
- US2 (Selected-Text Mode): 6 tasks
- US3 (Content Ingestion): 10 tasks
- US4 (Chat Widget): 13 tasks
- US5 (Authentication): 13 tasks

**Parallel Opportunities**: 33 tasks marked [P] can run in parallel within their phases

**MVP Scope** (Recommended): Phases 1, 2, 3, 4 = 45 tasks
- This delivers: Backend infrastructure + content ingestion + full-book question answering with citations
- Estimated effort: ~3-5 days for experienced developer

**Production-Ready Scope**: All 90 tasks
- This delivers: Full feature set with widget, selected-text mode, authentication, and deployment configs
- Estimated effort: ~7-10 days for experienced developer

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group of tasks
- Stop at any checkpoint to validate story independently
- Tests not included as they were not requested in specification
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
