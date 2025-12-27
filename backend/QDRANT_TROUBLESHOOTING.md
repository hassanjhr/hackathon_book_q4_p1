# Qdrant Cloud Connection Troubleshooting Guide

## Problem

Chatbot returns "I cannot find this information in the textbook" but works for PyTorch library questions.

## Root Cause

The backend cannot connect to Qdrant Cloud vector database, so textbook chunks aren't being retrieved.

---

## Diagnostic Steps

### Step 1: Use the Debug Endpoint

Visit this URL in your browser or use `curl`:

```bash
https://hassanjhr-rag-chatbot.hf.space/api/rag/debug
```

This will return a JSON response showing:

1. **Environment Variables** - Are they set correctly?
2. **Settings Loaded** - Did pydantic read them?
3. **Qdrant Service Status** - Is it marked as available?
4. **Connection Test** - Can it actually connect?
5. **Recommendations** - What to fix

### Example Debug Output

#### ✅ Working Configuration (Local):

```json
{
  "environment_variables": {
    "QDRANT_URL": "bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333",
    "QDRANT_URL_LENGTH": 75,
    "QDRANT_API_KEY": "SET (139 chars)",
    "QDRANT_COLLECTION_NAME": "textbook_content"
  },
  "settings_loaded_by_pydantic": {
    "qdrant_url": "bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333",
    "qdrant_url_length": 75,
    "has_api_key": true,
    "api_key_length": 139,
    "collection_name": "textbook_content"
  },
  "qdrant_service_status": {
    "is_available": true,
    "client_configured": true,
    "collection_name": "textbook_content"
  },
  "qdrant_connection_test": {
    "status": "SUCCESS",
    "collections": ["textbook_content"],
    "collection_count": 1,
    "target_collection_exists": true
  },
  "recommendations": [
    "✅ All checks passed!"
  ]
}
```

#### ❌ Failing Configuration (Hugging Face):

```json
{
  "environment_variables": {
    "QDRANT_URL": "NOT SET",  // ← Problem: env var not set!
    "QDRANT_URL_LENGTH": 0,
    "QDRANT_API_KEY": "NOT SET",
    "QDRANT_COLLECTION_NAME": "NOT SET"
  },
  "qdrant_connection_test": {
    "status": "FAILED",
    "error": "[Errno -2] Name or service not known",
    "error_type": "gaierror"
  },
  "recommendations": [
    "❌ QDRANT_URL environment variable is not set",
    "❌ QDRANT_API_KEY environment variable is not set",
    "🔥 DNS resolution failed - verify URL format"
  ]
}
```

---

## Common Issues & Fixes

### Issue 1: Environment Variables Not Set

**Symptoms:**
```json
{
  "environment_variables": {
    "QDRANT_URL": "NOT SET",
    "QDRANT_API_KEY": "NOT SET"
  }
}
```

**Fix:**

In Hugging Face Spaces → Settings → Environment Variables, add:

```
QDRANT_URL=bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.59gWvwZSWXPVmdhToxrgDzxANGCTYkdtJdSxfFlvFFw
QDRANT_COLLECTION_NAME=textbook_content
OPENAI_API_KEY=your-openai-key
```

**Important:**
- ❌ **DO NOT** include `https://` prefix
- ✅ **DO** include port `:6333`
- ✅ Restart the Space after adding variables

---

### Issue 2: DNS Resolution Failure

**Symptoms:**
```json
{
  "error": "[Errno -2] Name or service not known"
}
```

**Possible Causes:**
1. Wrong URL format (includes `https://` or missing port)
2. Typo in the URL
3. Network/firewall restrictions on Hugging Face

**Fix Attempts:**

1. **Verify URL Format:**
   - Correct: `bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333`
   - Wrong: `https://bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io:6333`
   - Wrong: `bbcdb8f8-bcf5-4129-acab-f133bbf378ce.europe-west3-0.gcp.cloud.qdrant.io` (missing port)

2. **Copy URL from Qdrant Cloud Dashboard:**
   - Go to https://cloud.qdrant.io
   - Click on your cluster
   - Look for "REST URL" or "Connection Details"
   - Copy the **exact** URL shown (without `https://`)

3. **Check for Extra Spaces/Characters:**
   - Environment variables can have invisible whitespace
   - Delete and re-create the variable if unsure

---

### Issue 3: Variables Set But Not Loaded

**Symptoms:**
```json
{
  "environment_variables": {
    "QDRANT_URL": "bbcdb8...:6333",  // ← Set
    "QDRANT_URL_LENGTH": 75
  },
  "settings_loaded_by_pydantic": {
    "qdrant_url": "NOT SET",  // ← Not loaded!
    "qdrant_url_length": 0
  }
}
```

**Fix:**

1. Verify environment variable **names** are exact (case-sensitive):
   - `QDRANT_URL` (not `Qdrant_URL` or `QDRANT_url`)
   - `QDRANT_API_KEY` (not `QDRANT_KEY`)

2. Restart the Hugging Face Space (factory reboot):
   - Settings → Factory Reboot

3. Check if Hugging Face uses a different env variable loading mechanism

---

### Issue 4: Connection Timeout

**Symptoms:**
```json
{
  "error": "TimeoutError: Connection timeout after 10 seconds"
}
```

**Possible Causes:**
- Hugging Face firewall blocks outbound connections to Qdrant Cloud
- Qdrant Cloud firewall blocks Hugging Face IPs

**Fix Attempts:**

1. **Check Qdrant Cloud Firewall:**
   - Go to Qdrant Cloud Dashboard
   - Check "Security" or "Access Control" settings
   - Ensure "Allow all IPs" or add Hugging Face IP ranges

2. **Try Alternative Deployment:**
   - Deploy backend to Railway.app instead
   - Railway has fewer network restrictions

---

## How the Code Works

### 1. Initialization (`qdrant_client.py:23-45`)

```python
def __init__(self):
    try:
        self.client = QdrantClient(
            url=settings.qdrant_url,        # From environment
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,              # Use REST API
            https=True,                      # Enable HTTPS
            timeout=30
        )
        self._ensure_collection_exists()
        self.is_available = True
        logger.info("✅ Qdrant connection successful")
    except Exception as e:
        logger.warning(f"⚠️ Qdrant unavailable: {str(e)[:100]}")
        self.is_available = False  # ← Service continues without Qdrant
```

**Key Point:** If Qdrant connection fails, the service starts but **marks Qdrant as unavailable**.

### 2. Search Behavior (`qdrant_client.py:97-137`)

```python
def search_vectors(self, query_vector, limit=20, score_threshold=None):
    try:
        results = self.client.search(...)
        return [...]
    except Exception as e:
        logger.error(f"Error searching vectors: {e}")
        return []  # ← Returns empty list on error
```

### 3. RAG Service Response (`rag_service.py:161-172`)

```python
search_results = qdrant_service.search_vectors(...)

if not search_results:  # ← Empty list from failed search
    return {
        "answer": "I couldn't find relevant information in the textbook to answer your question.",
        "sources": []
    }
```

**Result:** User sees "I cannot find this information in the textbook" because no chunks were retrieved.

---

## Verification Checklist

After applying fixes, verify in this order:

1. ✅ Visit `/api/rag/debug` - should show environment variables set
2. ✅ Check `is_available: true` in debug output
3. ✅ Check `qdrant_connection_test.status: "SUCCESS"`
4. ✅ Check `target_collection_exists: true`
5. ✅ Send test question via chatbot: "What is Physical AI?"
6. ✅ Verify response includes textbook sources (Module/Week citations)

---

## Alternative Solutions

### Option 1: Use Railway.app Instead

Railway has fewer network restrictions:

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Deploy
cd backend
railway up
```

Set environment variables in Railway dashboard.

### Option 2: Self-Host Qdrant

Instead of Qdrant Cloud, run Qdrant on the same platform as the backend:

```yaml
# docker-compose.yml
services:
  qdrant:
    image: qdrant/qdrant
    ports:
      - "6333:6333"
  backend:
    build: .
    environment:
      QDRANT_URL: qdrant:6333  # Use service name
```

### Option 3: Use Alternative Vector DB

Switch to Pinecone, Weaviate, or ChromaDB if Qdrant connectivity can't be resolved.

---

## Contact Support

If none of these solutions work:

1. **Hugging Face Forum:** https://discuss.huggingface.co/
   - Ask about outbound network connectivity restrictions

2. **Qdrant Discord:** https://discord.gg/qdrant
   - Share debug endpoint output for help

3. **Check Backend Logs:**
   - Hugging Face → Logs tab
   - Look for the exact error message during initialization

---

## Summary

**Most Common Issue:** Environment variables not set correctly on Hugging Face

**Quick Fix:**
1. Go to https://hassanjhr-rag-chatbot.hf.space/api/rag/debug
2. Check what the debug endpoint reports
3. Fix the issues listed in "recommendations"
4. Factory reboot the Space
5. Test again

**Expected Behavior:**
- Debug endpoint shows all green checks
- Chatbot answers textbook questions with Module/Week citations
