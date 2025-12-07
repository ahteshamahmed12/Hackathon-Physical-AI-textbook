# Troubleshooting Guide - RAG Documentation Chatbot

This guide covers common issues and their solutions when setting up and running the RAG-powered documentation chatbot.

## Table of Contents
1. [Backend Startup Issues](#backend-startup-issues)
2. [Missing Dependencies](#missing-dependencies)
3. [Environment Variable Issues](#environment-variable-issues)
4. [Qdrant Connection Issues](#qdrant-connection-issues)
5. [Ingestion Script Issues](#ingestion-script-issues)
6. [Frontend Integration Issues](#frontend-integration-issues)

---

## Backend Startup Issues

### Error: `ModuleNotFoundError: No module named 'anthropic'`

**Problem:** The Anthropic Python SDK is not installed.

**Solution:**
```bash
pip install anthropic
```

Or install all required dependencies:
```bash
pip install -r requirements.txt
```

### Error: `ModuleNotFoundError: No module named 'fastapi'` or similar

**Problem:** Required Python packages are missing.

**Solution:**
```bash
# Install all dependencies at once
pip install -r requirements.txt

# Or install individually:
pip install fastapi uvicorn qdrant-client anthropic langchain langchain-openai python-dotenv
```

### Error: `ValueError: Missing required environment variables`

**Problem:** The `.env` file is missing required API keys.

**Required Environment Variables:**
- `QDRANT_URL` - Qdrant vector database URL
- `QDRANT_API_KEY` - Qdrant authentication key
- `OPENAI_API_KEY` - OpenAI API key for embeddings
- `ANTHROPIC_API_KEY` - Anthropic API key for Claude

**Solution:**

1. **Check if `.env` file exists:**
   ```bash
   ls -la .env
   ```

2. **If missing, create from template:**
   ```bash
   cp .env.example .env
   ```

3. **Add your API keys to `.env`:**
   ```env
   QDRANT_URL=https://your-instance.qdrant.io
   QDRANT_API_KEY=your_qdrant_key
   OPENAI_API_KEY=sk-your_openai_key
   ANTHROPIC_API_KEY=sk-ant-your_anthropic_key
   ```

4. **Verify the file is not empty:**
   ```bash
   cat .env
   ```

---

## Missing Dependencies

### How to Install All Dependencies

**Method 1: Using requirements.txt (Recommended)**
```bash
pip install -r requirements.txt
```

**Method 2: Manual Installation**
```bash
pip install fastapi==0.109.0 \
            uvicorn[standard]==0.27.0 \
            qdrant-client==1.7.3 \
            anthropic==0.18.1 \
            langchain==0.1.5 \
            langchain-openai==0.0.5 \
            langchain-community==0.0.17 \
            openai==1.10.0 \
            python-dotenv==1.0.0
```

### Verify Installation

```bash
python -c "import fastapi, anthropic, qdrant_client; print('✓ All imports successful')"
```

---

## Environment Variable Issues

### Issue: Environment variables not loading

**Symptoms:**
- Error: "Missing required environment variables"
- Variables show as `None` in Python

**Common Causes:**
1. `.env` file in wrong location
2. Variables not properly formatted
3. File encoding issues

**Solutions:**

1. **Verify `.env` file location:**
   ```bash
   # Should be in project root (same directory as app.py)
   ls -la .env
   ```

2. **Check file format:**
   ```env
   # Correct format (no spaces around =)
   OPENAI_API_KEY=sk-your_key_here

   # Incorrect formats:
   OPENAI_API_KEY = sk-your_key_here  ❌ (spaces)
   OPENAI_API_KEY                       ❌ (no value)
   ```

3. **Test environment variable loading:**
   ```python
   from dotenv import load_dotenv
   import os

   load_dotenv()
   print("OPENAI_API_KEY:", os.getenv("OPENAI_API_KEY"))
   ```

---

## Qdrant Connection Issues

### Error: `Collection 'docusaurus_docs' not found`

**Problem:** The vector database collection hasn't been created yet.

**Solution:**
```bash
# Run the ingestion script to create and populate the collection
python ingest.py
```

### Error: `Unable to connect to Qdrant`

**Problem:** Qdrant credentials are incorrect or the service is unavailable.

**Debugging Steps:**

1. **Verify Qdrant credentials:**
   ```bash
   cat .env | grep QDRANT
   ```

2. **Test Qdrant connection:**
   ```python
   from qdrant_client import QdrantClient
   import os
   from dotenv import load_dotenv

   load_dotenv()

   client = QdrantClient(
       url=os.getenv("QDRANT_URL"),
       api_key=os.getenv("QDRANT_API_KEY")
   )

   # List collections
   collections = client.get_collections()
   print("Collections:", [c.name for c in collections.collections])
   ```

3. **Check Qdrant Cloud Dashboard:**
   - Verify cluster is running
   - Check API key is active
   - Confirm URL is correct

---

## Ingestion Script Issues

### Error: `No markdown files found in ./docs`

**Problem:** The `docs` directory is empty or in the wrong location.

**Solution:**

1. **Check docs directory:**
   ```bash
   ls -la docs/
   ```

2. **Verify markdown files exist:**
   ```bash
   find docs/ -name "*.md" -o -name "*.mdx"
   ```

3. **If docs are in different location, update `ingest.py`:**
   ```python
   # Change this line in ingest.py:
   docs_path = "./docs"  # Update to your docs location
   ```

### Error: `OpenAI API error: Invalid API key`

**Problem:** OpenAI API key is incorrect or expired.

**Solution:**

1. **Verify API key format:**
   - Should start with `sk-`
   - Example: `sk-proj-abc123...`

2. **Check API key on OpenAI dashboard:**
   - Visit: https://platform.openai.com/api-keys
   - Verify key is active
   - Create new key if needed

3. **Update `.env` file:**
   ```env
   OPENAI_API_KEY=sk-your_new_key_here
   ```

### Ingestion takes too long

**Problem:** Large documentation set causing slow ingestion.

**Optimization Options:**

1. **Batch processing** (already implemented in `ingest.py`)
2. **Reduce chunk size:**
   ```python
   # In ingest.py, modify:
   text_splitter = RecursiveCharacterTextSplitter(
       chunk_size=500,   # Smaller chunks
       chunk_overlap=100
   )
   ```

3. **Filter specific files:**
   ```python
   # Only process certain directories
   markdown_files = glob.glob(f"{docs_path}/important/**/*.md", recursive=True)
   ```

---

## Frontend Integration Issues

### ChatWidget not displaying

**Problem:** Component doesn't render or shows blank.

**Debugging Steps:**

1. **Check browser console for errors:**
   - Press F12 → Console tab
   - Look for import errors or React errors

2. **Verify component import:**
   ```jsx
   // Correct import
   import ChatWidget from '@site/src/components/ChatWidget';

   // Common mistakes:
   import ChatWidget from './ChatWidget';  ❌
   import { ChatWidget } from '@site/src/components/ChatWidget';  ❌
   ```

3. **Check file location:**
   ```bash
   ls -la src/components/ChatWidget.js
   ```

4. **Restart Docusaurus dev server:**
   ```bash
   npm start
   ```

### Error: "Failed to fetch" in chat

**Problem:** Cannot connect to backend.

**Solutions:**

1. **Verify backend is running:**
   ```bash
   curl http://localhost:8000/health
   ```

2. **Check backend port:**
   - Default: `http://localhost:8000`
   - Update `ChatWidget.js` if using different port:
     ```javascript
     const response = await fetch('http://localhost:YOUR_PORT/chat', {
       // ...
     });
     ```

3. **Verify CORS settings in `app.py`:**
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://localhost:3000"],  # Match your frontend port
       # ...
   )
   ```

### Error: "Collection not found" from backend

**Problem:** Qdrant collection not created.

**Solution:**
```bash
# Run ingestion script first
python ingest.py

# Then restart backend
python app.py
```

---

## API Key Setup Guide

### Getting OpenAI API Key

1. Visit: https://platform.openai.com/api-keys
2. Sign in or create account
3. Click "Create new secret key"
4. Copy the key (starts with `sk-`)
5. Add to `.env`:
   ```env
   OPENAI_API_KEY=sk-your_key_here
   ```

### Getting Anthropic API Key

1. Visit: https://console.anthropic.com/
2. Sign in or create account
3. Navigate to API Keys section
4. Click "Create Key"
5. Copy the key (starts with `sk-ant-`)
6. Add to `.env`:
   ```env
   ANTHROPIC_API_KEY=sk-ant-your_key_here
   ```

### Getting Qdrant Credentials

1. Visit: https://cloud.qdrant.io/
2. Sign in or create account
3. Create a cluster (free tier available)
4. Copy cluster URL and API key
5. Add to `.env`:
   ```env
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your_api_key_here
   ```

---

## Quick Diagnostic Commands

### Check all dependencies:
```bash
pip list | grep -E "(fastapi|anthropic|qdrant|langchain|openai)"
```

### Test backend import:
```bash
python -c "import app; print('✓ Backend imports successfully')"
```

### Test Qdrant connection:
```bash
python -c "from qdrant_client import QdrantClient; import os; from dotenv import load_dotenv; load_dotenv(); client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print('✓ Qdrant connected:', client.get_collections())"
```

### Test OpenAI connection:
```bash
python -c "from langchain_openai import OpenAIEmbeddings; import os; from dotenv import load_dotenv; load_dotenv(); e = OpenAIEmbeddings(openai_api_key=os.getenv('OPENAI_API_KEY')); print('✓ OpenAI connected')"
```

### Check backend health:
```bash
curl http://localhost:8000/health
```

---

## Complete Setup Checklist

Use this checklist to ensure everything is properly configured:

- [ ] Python 3.8+ installed
- [ ] All dependencies installed (`pip install -r requirements.txt`)
- [ ] `.env` file created with all required keys
- [ ] OpenAI API key added and valid
- [ ] Anthropic API key added and valid
- [ ] Qdrant cluster created and credentials added
- [ ] Documentation files exist in `./docs` directory
- [ ] Ingestion script run successfully (`python ingest.py`)
- [ ] Backend starts without errors (`python app.py`)
- [ ] Backend health check passes (`curl http://localhost:8000/health`)
- [ ] Frontend starts (`npm start`)
- [ ] ChatWidget component renders
- [ ] Test query returns expected response

---

## Still Having Issues?

If you've tried all the above solutions and still have issues:

1. **Check the error logs carefully** - The specific error message often contains the solution
2. **Verify Python version**: `python --version` (should be 3.8+)
3. **Check file permissions** - Ensure you have read/write access
4. **Try in a fresh virtual environment**:
   ```bash
   python -m venv fresh_venv
   source fresh_venv/bin/activate  # On Windows: fresh_venv\Scripts\activate
   pip install -r requirements.txt
   ```
5. **Review the specification**: `specs/001-rag-doc-chatbot/spec.md`
6. **Check README files**: `README_BACKEND.md` and `src/components/README_CHATWIDGET.md`

---

## Common Command Reference

```bash
# Install dependencies
pip install -r requirements.txt

# Run ingestion
python ingest.py

# Start backend
python app.py
# OR
uvicorn app:app --reload --port 8000

# Start frontend
npm start

# Test backend
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What GPU do I need?"}'
```

---

For additional help, refer to:
- `README_BACKEND.md` - Backend documentation
- `src/components/README_CHATWIDGET.md` - Frontend documentation
- `specs/001-rag-doc-chatbot/spec.md` - Feature specification
