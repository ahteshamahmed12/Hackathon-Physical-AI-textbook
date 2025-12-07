---
id: 002
title: Implement FastAPI Backend with Anthropic Claude
stage: green
date_iso: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-doc-chatbot
branch: 001-rag-doc-chatbot
user: ahtesham ahmed
command: /sp.implement
labels: ["implementation", "fastapi", "backend", "anthropic", "qdrant", "rag"]
links:
  spec: specs/001-rag-doc-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - app.py
  - requirements.txt
  - README_BACKEND.md
  - .gitignore
tests_yaml: []
---

## Prompt

```text
task-3
> Create the full FastAPI application code **`app.py`**.
>
> Use the following libraries: `fastapi`, `uvicorn`, `qdrant-client`, `anthropic`, `pydantic`, `openai`.
>
> **Key steps to include in the code:**
> 1. Configure **CORS** for `http://localhost:3000`.
> 2. Define a Pydantic model `ChatQuery(query: str)`.
> 3. Create the `POST /chat` endpoint.
> 4. In the endpoint, retrieve the **top 3** chunks from Qdrant based on the user's query.
> 5. Construct a **system prompt** that clearly instructs the **Anthropic Claude** model to *only* answer based on the provided RAG context.
> 6. Call the **Anthropic API** with the constructed RAG prompt and return the final text response.
```

## Response

Successfully implemented Task 3 - FastAPI backend application with complete RAG functionality using Anthropic Claude API.

**Implementation Summary:**

1. **Created `app.py`** - Full FastAPI application with:
   - CORS middleware configured for `http://localhost:3000` (Docusaurus frontend)
   - Pydantic models: `ChatQuery` (with validation) and `ChatResponse`
   - Three endpoints:
     - `GET /` - Root health check
     - `GET /health` - Detailed health check with Qdrant connection status
     - `POST /chat` - Main RAG endpoint
   - Complete error handling with appropriate HTTP status codes (400, 500, 503)
   - Environment variable validation on startup
   - Comprehensive inline documentation

2. **POST /chat Endpoint Implementation:**
   - Step 1: Generates query embedding using OpenAI Embeddings API (`text-embedding-3-small`)
   - Step 2: Performs similarity search in Qdrant to retrieve top-3 document chunks
   - Step 3: Extracts content and source metadata from search results
   - Step 4: Constructs RAG system prompt with:
     - Clear instructions to answer only from provided context
     - All 3 retrieved document chunks with source citations
     - User's original query
     - Fallback message when no relevant context found
   - Step 5: Calls Anthropic Claude API (`claude-3-5-sonnet-20241022`)
   - Step 6: Returns structured JSON response with AI answer and sources count

3. **Error Handling:**
   - Missing environment variables → Detailed error message listing missing vars
   - Query embedding failures → 500 error with specific details
   - Collection not found → 503 error with helpful message to run ingestion script
   - Qdrant search failures → Appropriate 500/503 errors
   - Anthropic API errors → 503 error with API-specific details
   - Invalid request payloads → 400 error via Pydantic validation
   - All unexpected errors → Generic 500 error with details

4. **Supporting Files:**
   - **requirements.txt** - All Python dependencies with pinned versions:
     - fastapi==0.109.0
     - uvicorn[standard]==0.27.0
     - qdrant-client==1.7.3
     - anthropic==0.18.1
     - langchain==0.1.5 (for OpenAI embeddings)
     - python-dotenv==1.0.0
   - **README_BACKEND.md** - Comprehensive documentation:
     - Installation instructions
     - API endpoint documentation with examples
     - Testing methods (curl, Python requests, Swagger UI)
     - Error handling reference
     - Architecture diagram
     - Troubleshooting guide
     - Production deployment considerations

5. **Security Improvements:**
   - Added `.env` to `.gitignore` to prevent credential exposure
   - Validated all environment variables on startup
   - Input validation with Pydantic (min/max length, whitespace checking)
   - Proper error messages without exposing internal details

**Key Features Implemented:**

✅ CORS configured for `http://localhost:3000`
✅ Pydantic model `ChatQuery` with validation
✅ POST /chat endpoint with complete RAG pipeline
✅ Retrieves exactly top-3 chunks from Qdrant
✅ System prompt instructs Claude to answer only from context
✅ Anthropic Claude API integration (`claude-3-5-sonnet-20241022`)
✅ Returns JSON response with AI answer
✅ Comprehensive error handling (400, 500, 503)
✅ Health check endpoints
✅ Production-ready with uvicorn entry point

**Technical Alignment:**

- Uses same embedding model as `ingest.py` (`text-embedding-3-small`, vector size 1536)
- Matches specification requirements from `specs/001-rag-doc-chatbot/spec.md`:
  - FR2.1-FR2.10 (all backend functional requirements)
  - NFR3.1-NFR3.5 (security requirements)
  - Error handling per SC7 (clear, actionable messages)

**Testing:**

The application can be tested via:
1. Interactive Swagger UI at `http://localhost:8000/docs`
2. curl commands (examples in README_BACKEND.md)
3. Python requests library
4. Integration with frontend ChatWidget.js

**Next Steps:**

To run the backend:
```bash
# Install dependencies
pip install -r requirements.txt

# Ensure .env has all required keys
# QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY, ANTHROPIC_API_KEY

# Run the server
python app.py
# OR
uvicorn app:app --reload
```

Backend is ready for integration with the frontend ChatWidget component (Task 4).
