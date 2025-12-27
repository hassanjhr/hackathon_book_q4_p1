# ChatKit Integration - Deployment & Testing Guide

## Overview

This guide provides step-by-step instructions for deploying and testing the ChatKit integration for the AI-Native Textbook RAG chatbot.

## What Was Implemented

### Backend Components

1. **Database Migration** (`migrations/002_chatkit_schema.sql`)
   - Thread metadata storage
   - Thread items (messages, tool calls, tasks, etc.)
   - Pagination cursors

2. **ChatKit Store** (`src/services/chatkit_store.py`)
   - Postgres-backed Store implementation
   - Thread and message persistence
   - Pagination support with Page[ThreadItem]

3. **ChatKit Server** (`src/services/chatkit_server.py`)
   - Wraps existing RAG service
   - Streaming response generation
   - Multi-source context (textbook + library docs)
   - Source citation formatting

4. **ChatKit API Routes** (`src/api/chatkit_routes.py`)
   - `POST /api/chatkit/chat` - Streaming chat endpoint
   - `GET /api/chatkit/threads/{user_id}` - List threads
   - `GET /api/chatkit/threads/{thread_id}/messages` - Get messages
   - `DELETE /api/chatkit/threads/{thread_id}` - Delete thread
   - `GET /api/chatkit/health` - Health check

### Frontend Components

1. **ChatKitWidget** (`my-website/src/components/ChatKitWidget/`)
   - Modern React component with hooks
   - SSE (Server-Sent Events) streaming support
   - Real-time response display
   - Source citation rendering
   - Mobile-responsive design
   - Dark mode support via Docusaurus theme

---

## Deployment Steps

### 1. Backend Setup

#### Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

**Note**: The `openai-chatkit` package may not be publicly available yet. The implementation is designed to work without it, using custom implementations.

#### Run Database Migration

```bash
# Connect to your Neon Postgres database
psql "$DATABASE_URL" -f migrations/002_chatkit_schema.sql
```

Expected output:
```
CREATE TABLE
CREATE TABLE
CREATE TABLE
CREATE INDEX
...
NOTICE: ChatKit schema migration completed successfully
```

#### Verify Configuration

Ensure `.env` has the required settings:

```bash
# OpenAI Configuration (required)
OPENAI_API_KEY=sk-...

# Neon Postgres (required)
DATABASE_URL=postgresql://...

# Qdrant (required)
QDRANT_URL=https://...
QDRANT_API_KEY=...

# Context7 (optional but recommended)
CONTEXT7_ENABLED=true
CONTEXT7_MAX_RESULTS=2

# ChatKit (optional, uses defaults if not set)
CHATKIT_ENABLED=true
CHATKIT_MAX_THREADS_PER_USER=100
```

#### Start Backend Server

```bash
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

### 2. Frontend Setup

#### Install Dependencies

```bash
cd my-website
npm install
```

#### Configure API URL

Create or update `my-website/.env.local`:

```bash
DOCUSAURUS_API_URL=http://localhost:8000
```

For production:
```bash
DOCUSAURUS_API_URL=https://your-backend-domain.com
```

#### Start Frontend

```bash
cd my-website
npm start
```

Expected output:
```
[SUCCESS] Serving at http://localhost:3000
```

---

## Testing Guide

### Test 1: Backend Health Check

Verify all services are operational:

```bash
curl http://localhost:8000/api/chatkit/health | python3 -m json.tool
```

Expected response:
```json
{
  "status": "healthy",
  "chatkit_server": "operational",
  "rag_service": "healthy",
  "store": "operational"
}
```

### Test 2: Non-Streaming Chat

Test synchronous chat endpoint:

```bash
curl -X POST http://localhost:8000/api/chatkit/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is Physical AI?",
    "user_id": "test-user",
    "stream": false
  }' | python3 -m json.tool
```

Expected response structure:
```json
{
  "thread_id": "uuid-here",
  "answer": "Physical AI is...",
  "sources": [
    {
      "type": "textbook",
      "module": "Module 0",
      "week": "Week 1",
      "file": "introduction.md",
      "relevance_score": 0.89
    }
  ],
  "role": "assistant"
}
```

### Test 3: Streaming Chat

Test SSE streaming endpoint:

```bash
curl -X POST http://localhost:8000/api/chatkit/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "How do I create a PyTorch tensor?",
    "user_id": "test-user",
    "use_library_docs": true,
    "stream": true
  }'
```

Expected output (SSE format):
```
event: thread.created
data: {"thread_id": "uuid", "user_id": "test-user", ...}

event: thread.message.delta
data: {"delta": "To create", "role": "assistant"}

event: thread.message.delta
data: {"delta": " a PyTorch", "role": "assistant"}

...

event: thread.message.completed
data: {"content": "To create a PyTorch tensor...", "sources": [...]}

event: thread.run.completed
data: {"thread_id": "uuid", "status": "completed"}
```

### Test 4: Context7 Library Documentation

Test library question with Context7:

```bash
curl -X POST http://localhost:8000/api/chatkit/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "How do I create a ROS2 node?",
    "user_id": "test-user",
    "use_library_docs": true,
    "stream": false
  }' | python3 -m json.tool
```

Expected: Response should include sources with `"type": "library_docs"` if Context7 is configured.

### Test 5: Thread Management

#### List Threads

```bash
curl http://localhost:8000/api/chatkit/threads/test-user | python3 -m json.tool
```

#### Get Thread Messages

```bash
# Replace {thread_id} with actual thread ID from previous response
curl "http://localhost:8000/api/chatkit/threads/{thread_id}/messages?limit=10" | python3 -m json.tool
```

#### Delete Thread

```bash
curl -X DELETE "http://localhost:8000/api/chatkit/threads/{thread_id}"
```

### Test 6: Frontend Widget

1. Open browser to `http://localhost:3000`
2. Click the floating chat button (💬) in bottom-right corner
3. Chat panel should open
4. Try starter prompts:
   - "What is Physical AI?"
   - "How do I create a PyTorch tensor?"
5. Verify:
   - Real-time streaming responses
   - Source citations display
   - Mobile responsiveness
   - Dark mode compatibility

---

## Troubleshooting

### Issue: "Failed to fetch" in frontend

**Cause**: CORS not configured or backend not running

**Solution**:
1. Verify backend is running on port 8000
2. Check browser console for CORS errors
3. Verify `.env.local` has correct API URL

### Issue: "Database error" in responses

**Cause**: Migration not run or DATABASE_URL incorrect

**Solution**:
```bash
# Verify database connection
psql "$DATABASE_URL" -c "SELECT COUNT(*) FROM chatkit_threads;"

# Re-run migration if needed
psql "$DATABASE_URL" -f migrations/002_chatkit_schema.sql
```

### Issue: No library documentation in responses

**Cause**: Context7 not enabled or MCP server not available

**Solution**:
1. Verify `.env` has `CONTEXT7_ENABLED=true`
2. Check if library questions trigger detection (see logs)
3. Context7 requires MCP server - may need manual testing

### Issue: SSE stream disconnects

**Cause**: Nginx buffering or proxy timeout

**Solution**:
Add to nginx config:
```nginx
proxy_buffering off;
proxy_read_timeout 300s;
```

---

## Performance Optimization

### Database Indexes

The migration creates these indexes for performance:
- `idx_chatkit_threads_user` - Thread lookups by user
- `idx_chatkit_items_thread` - Message retrieval
- `idx_chatkit_items_sequence` - Pagination

### Cleanup Tasks

Run periodically to clean expired cursors:

```sql
SELECT cleanup_expired_pagination_cursors();
```

### Monitoring Queries

```sql
-- Count threads per user
SELECT user_id, COUNT(*) as thread_count
FROM chatkit_threads
GROUP BY user_id
ORDER BY thread_count DESC
LIMIT 10;

-- Count messages per thread
SELECT thread_id, COUNT(*) as message_count
FROM chatkit_thread_items
GROUP BY thread_id
ORDER BY message_count DESC
LIMIT 10;

-- Average response time (if timestamps added)
SELECT AVG(EXTRACT(EPOCH FROM (created_at - LAG(created_at) OVER (PARTITION BY thread_id ORDER BY created_at))))
FROM chatkit_thread_items
WHERE item_type = 'message';
```

---

## API Endpoints Reference

### Chat

**POST** `/api/chatkit/chat`

Request:
```json
{
  "message": "string",
  "thread_id": "uuid (optional)",
  "user_id": "string",
  "use_library_docs": true,
  "stream": true
}
```

Response (non-streaming):
```json
{
  "thread_id": "uuid",
  "answer": "string",
  "sources": [...],
  "role": "assistant"
}
```

Response (streaming): SSE events

### Thread Management

- **GET** `/api/chatkit/threads/{user_id}?limit=50&offset=0`
- **GET** `/api/chatkit/threads/{thread_id}/messages?limit=50&cursor=123`
- **DELETE** `/api/chatkit/threads/{thread_id}`

### Health

- **GET** `/api/chatkit/health`

### Test

- **POST** `/api/chatkit/test?question=...&stream=false`

---

## Next Steps

1. **Production Deployment**:
   - Set up environment variables in production
   - Run database migration on production database
   - Configure CORS for production domain
   - Set up reverse proxy (nginx/Caddy)

2. **User Authentication**:
   - Integrate with Better-Auth (already configured in `.env`)
   - Replace `user_id = "anonymous"` with actual user IDs
   - Add thread ownership validation

3. **Advanced Features**:
   - Add custom ChatKit tools (citation lookup, module search)
   - Implement user feedback collection
   - Add analytics and usage tracking
   - Create admin dashboard for thread management

4. **Monitoring**:
   - Set up error tracking (Sentry)
   - Add performance monitoring (New Relic, DataDog)
   - Create alerting for API failures

---

## Success Criteria

✅ Backend ChatKit endpoints respond successfully
✅ Database migration completes without errors
✅ Frontend chat widget loads and displays
✅ Messages stream in real-time
✅ Source citations appear correctly
✅ Thread persistence works (refresh page, messages remain)
✅ Context7 library docs enhance responses (when available)
✅ Mobile responsiveness verified
✅ Dark mode compatibility confirmed

---

## Additional Resources

- **Plan Document**: `/home/hassanjhr/.claude/plans/vectorized-giggling-starfish.md`
- **RAG Service**: `backend/src/services/rag_service.py`
- **Context7 Service**: `backend/src/services/context7_service.py`
- **Database Schema**: `backend/migrations/002_chatkit_schema.sql`

---

**Questions or Issues?**

Check the implementation files for detailed comments and documentation. All components are thoroughly documented with type hints and docstrings.
