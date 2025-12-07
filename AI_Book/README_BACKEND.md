# FastAPI Backend - RAG Documentation Chatbot

This is the FastAPI backend service for the RAG-powered documentation chatbot. It provides REST API endpoints for answering user queries based on documentation content using Retrieval-Augmented Generation with **Google Gemini embeddings** and **Anthropic Claude** for response generation.

## Features

- **POST /chat** - Main endpoint for processing user queries
- **GET /** - Root endpoint (health check)
- **GET /health** - Detailed health check with Qdrant connection status
- CORS enabled for `http://localhost:3000` (Docusaurus frontend)
- Retrieves top-3 relevant documentation chunks from Qdrant
- Uses **Google Gemini** for embeddings (768-dimensional vectors)
- Uses **Anthropic Claude** API for response generation
- Comprehensive error handling with informative error messages

## Prerequisites

1. **Python 3.8+** installed
2. **Environment variables** configured in `.env` file:
   ```env
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   GEMINI_API_KEY=your_gemini_api_key
   ANTHROPIC_API_KEY=your_anthropic_api_key
   ```

3. **Qdrant collection** populated with documentation:
   - Run `python ingest.py` first to create and populate the `docusaurus_docs` collection with Gemini embeddings

## Installation

1. **Create a virtual environment** (recommended):
   ```bash
   python -m venv venv

   # On Windows:
   venv\Scripts\activate

   # On macOS/Linux:
   source venv/bin/activate
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## Running the Server

### Option 1: Using Python directly
```bash
python app.py
```

### Option 2: Using uvicorn command
```bash
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

The server will start on `http://localhost:8000`

## API Endpoints

### 1. Root Endpoint
```bash
GET http://localhost:8000/
```

**Response:**
```json
{
  "message": "Documentation Chatbot API is running",
  "status": "healthy",
  "version": "1.0.0"
}
```

### 2. Health Check
```bash
GET http://localhost:8000/health
```

**Response:**
```json
{
  "status": "healthy",
  "qdrant": "connected",
  "collections": ["docusaurus_docs"]
}
```

### 3. Chat Endpoint
```bash
POST http://localhost:8000/chat
Content-Type: application/json

{
  "query": "What GPU do I need?"
}
```

**Response:**
```json
{
  "response": "Based on the documentation, you need an RTX 4070 Ti GPU...",
  "sources_count": 3
}
```

## Testing the API

### Using curl:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What GPU do I need?"}'
```

### Using Python requests:
```python
import requests

response = requests.post(
    "http://localhost:8000/chat",
    json={"query": "What GPU do I need?"}
)

print(response.json())
```

### Using the interactive API documentation:
Open your browser and visit:
- **Swagger UI:** http://localhost:8000/docs
- **ReDoc:** http://localhost:8000/redoc

## Error Handling

The API returns appropriate HTTP status codes:

- **200** - Success
- **400** - Bad request (invalid query format)
- **500** - Internal server error (API failures, processing errors)
- **503** - Service unavailable (Qdrant or Anthropic API unreachable)

Example error response:
```json
{
  "detail": "Collection 'docusaurus_docs' not found. Please run the ingestion script (python ingest.py) first."
}
```

## Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `QDRANT_URL` | Qdrant vector database URL | Yes |
| `QDRANT_API_KEY` | Qdrant API authentication key | Yes |
| `OPENAI_API_KEY` | OpenAI API key for embeddings | Yes |
| `ANTHROPIC_API_KEY` | Anthropic API key for Claude | Yes |

## Architecture

```
User Query → FastAPI Backend
    ↓
1. Embed query (OpenAI Embeddings)
    ↓
2. Similarity search (Qdrant - top 3 chunks)
    ↓
3. Construct RAG prompt with context
    ↓
4. Generate response (Anthropic Claude API)
    ↓
5. Return JSON response
```

## Development

### Auto-reload
The server runs with `--reload` flag by default, so changes to `app.py` will automatically restart the server.

### Logging
The server logs all requests and errors to the console with INFO level logging.

### Adding New Endpoints
Add new endpoint functions to `app.py` using FastAPI decorators:

```python
@app.get("/my-endpoint")
async def my_endpoint():
    return {"message": "Hello"}
```

## Troubleshooting

### Collection not found error
- **Solution:** Run `python ingest.py` to create and populate the collection

### API authentication errors
- **Solution:** Verify all API keys in `.env` file are correct and active

### CORS errors from frontend
- **Solution:** Check that frontend is running on `http://localhost:3000`
- If using a different port, update `allow_origins` in `app.py`

### Import errors
- **Solution:** Ensure virtual environment is activated and all dependencies are installed:
  ```bash
  pip install -r requirements.txt
  ```

## Production Considerations

For production deployment:

1. **Remove `--reload` flag** from uvicorn
2. **Use a production-grade ASGI server** (e.g., gunicorn with uvicorn workers)
3. **Set up proper secrets management** (not `.env` files)
4. **Configure proper CORS origins** for production domain
5. **Add rate limiting** to prevent abuse
6. **Set up monitoring and logging** infrastructure
7. **Use HTTPS** for all API communications

## License

This backend is part of the RAG Documentation Chatbot project.
