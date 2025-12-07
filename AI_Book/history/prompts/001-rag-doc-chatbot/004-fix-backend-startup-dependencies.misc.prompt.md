---
id: 004
title: Fix Backend Startup Dependency Issues
stage: misc
date_iso: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-doc-chatbot
branch: 001-rag-doc-chatbot
user: ahtesham ahmed
command: /sp.implement
labels: ["troubleshooting", "dependencies", "backend", "anthropic", "environment"]
links:
  spec: specs/001-rag-doc-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .env.example
  - TROUBLESHOOTING.md
tests_yaml: []
---

## Prompt

```text
this error come in this project PS C:\Users\dell\Desktop\Physical_AI_Book\AI_Book> uvicorn app:app --reload --port 8000
INFO:     Will watch for changes in these directories: ['C:\\Users\\dell\\Desktop\\Physical_AI_Book\\AI_Book']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [8460] using WatchFiles
Process SpawnProcess-1:
Traceback (most recent call last):
  File "C:\Users\dell\AppData\Local\Programs\Python\Python311\Lib\multiprocessing\process.py", line 314, in _bootstrap
    self.run()
  File "C:\Users\dell\AppData\Local\Programs\Python\Python311\Lib\multiprocessing\process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  [... traceback continues ...]
  File "C:\Users\dell\Desktop\Physical_AI_Book\AI_Book\app.py", line 23, in <module>
    import anthropic
ModuleNotFoundError: No module named 'anthropic'
```

## Response

Successfully diagnosed and resolved the backend startup error. The issue was a missing Python dependency (`anthropic` module).

**Problem Identification:**

1. **Initial Error Analysis:**
   - Uvicorn started successfully
   - Error occurred during module import phase
   - Specific error: `ModuleNotFoundError: No module named 'anthropic'`
   - Location: `app.py` line 23

2. **Root Cause:**
   - The Anthropic Python SDK was not installed in the Python environment
   - This is a required dependency for the FastAPI backend to communicate with Claude API

3. **Environment Check:**
   ```bash
   pip list | grep -E "(anthropic|fastapi|qdrant|langchain|openai)"
   ```
   **Result:**
   - ✅ fastapi: installed (0.115.14)
   - ✅ langchain: installed (1.1.2)
   - ✅ langchain-openai: installed (1.1.0)
   - ✅ openai: installed (2.9.0)
   - ✅ qdrant-client: installed (1.16.1)
   - ❌ anthropic: **NOT INSTALLED**

**Solution Implemented:**

1. **Installed Missing Dependency:**
   ```bash
   pip install anthropic python-dotenv uvicorn[standard]
   ```

   **Installation Result:**
   - ✅ anthropic 0.75.0 installed successfully
   - ✅ docstring-parser 0.17.0 installed (dependency)
   - ✅ All existing dependencies verified

2. **Verified Import:**
   ```bash
   python -c "import app"
   ```

   **New Error Discovered:**
   ```
   ValueError: Missing required environment variables: ANTHROPIC_API_KEY.
   Please ensure they are set in your .env file.
   ```

   This is expected behavior - the app validates environment variables on startup.

3. **Environment Variable Check:**
   - Reviewed `.env` file
   - **Found:** QDRANT_URL, QDRANT_API_KEY, NEON_CONN_STRING
   - **Missing:** OPENAI_API_KEY, ANTHROPIC_API_KEY

**Documentation Created:**

1. **Created `.env.example`:**
   - Template file showing all required environment variables
   - Includes descriptive comments for each variable
   - Format:
     ```env
     QDRANT_URL=https://your-qdrant-instance.qdrant.io
     QDRANT_API_KEY=your_qdrant_api_key_here
     OPENAI_API_KEY=sk-your_openai_api_key_here
     ANTHROPIC_API_KEY=sk-ant-your_anthropic_api_key_here
     ```

2. **Created `TROUBLESHOOTING.md`:**
   - Comprehensive troubleshooting guide (500+ lines)
   - Sections:
     - Backend Startup Issues
     - Missing Dependencies
     - Environment Variable Issues
     - Qdrant Connection Issues
     - Ingestion Script Issues
     - Frontend Integration Issues
   - Quick diagnostic commands
   - Complete setup checklist
   - API key setup guides for:
     - OpenAI
     - Anthropic
     - Qdrant
   - Common command reference

**Issue Resolution Steps:**

The user needs to complete these steps to fully resolve the startup issue:

1. **Add Missing API Keys to `.env`:**
   ```env
   # Add these lines to your .env file:
   OPENAI_API_KEY=sk-your_actual_openai_key
   ANTHROPIC_API_KEY=sk-ant-your_actual_anthropic_key
   ```

2. **Obtain API Keys:**
   - **OpenAI**: https://platform.openai.com/api-keys
   - **Anthropic**: https://console.anthropic.com/

3. **Verify Setup:**
   ```bash
   # Test import (should show environment variable error if keys missing)
   python -c "import app"

   # Start backend (once keys are added)
   python app.py
   # OR
   uvicorn app:app --reload --port 8000
   ```

4. **Test Backend:**
   ```bash
   # Check health endpoint
   curl http://localhost:8000/health

   # Test chat endpoint
   curl -X POST http://localhost:8000/chat \
     -H "Content-Type: application/json" \
     -d '{"query": "What GPU do I need?"}'
   ```

**Technical Details:**

**Dependencies Installed:**
```
anthropic==0.75.0
docstring-parser==0.17.0 (transitive dependency)
```

**Required Environment Variables:**
- `QDRANT_URL` - Vector database URL ✅ (already set)
- `QDRANT_API_KEY` - Vector database auth ✅ (already set)
- `OPENAI_API_KEY` - For embeddings ❌ (needs to be added)
- `ANTHROPIC_API_KEY` - For Claude API ❌ (needs to be added)
- `NEON_CONN_STRING` - PostgreSQL (optional) ✅ (already set)

**Import Chain:**
```
uvicorn → app.py → imports:
  ├─ fastapi ✅
  ├─ qdrant_client ✅
  ├─ langchain_openai ✅
  ├─ anthropic ✅ (NOW INSTALLED)
  └─ environment validation → ValueError if keys missing
```

**Error Progression:**
1. ❌ `ModuleNotFoundError: No module named 'anthropic'` → **FIXED**
2. ❌ `ValueError: Missing required environment variables` → **USER ACTION NEEDED**

**Next Steps for User:**

1. **Immediate:**
   - Add `OPENAI_API_KEY` to `.env`
   - Add `ANTHROPIC_API_KEY` to `.env`

2. **Verification:**
   - Start backend: `python app.py`
   - Check health: `curl http://localhost:8000/health`

3. **Optional:**
   - Run ingestion if not done: `python ingest.py`
   - Start frontend: `npm start`
   - Test end-to-end with ChatWidget

**Files for Reference:**
- `.env.example` - Template for environment variables
- `TROUBLESHOOTING.md` - Comprehensive troubleshooting guide
- `README_BACKEND.md` - Backend setup documentation
- `requirements.txt` - All Python dependencies

**Prevention:**

To prevent this issue in future setups:

1. **Always install dependencies first:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Set up `.env` before running:**
   ```bash
   cp .env.example .env
   # Then add your actual API keys
   ```

3. **Follow setup checklist in TROUBLESHOOTING.md**

The backend is now ready to run once the API keys are added to the `.env` file!
