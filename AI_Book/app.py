"""
FastAPI Backend for RAG-Powered Documentation Chatbot with Google Gemini

This application provides a REST API endpoint for answering user queries
based on documentation content using Retrieval-Augmented Generation (RAG)
with Google's Gemini API for both embeddings and response generation.

Key Components:
- POST /chat endpoint for user queries
- Qdrant vector database integration for semantic search
- Google Gemini embeddings for query vectorization
- Anthropic Claude API for response generation
- CORS middleware for frontend integration
"""

import os
from typing import Optional
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, validator
from qdrant_client import QdrantClient
import google.generativeai as genai
import anthropic

# Load environment variables from .env file
load_dotenv()

# Initialize FastAPI application
app = FastAPI(
    title="Documentation Chatbot API",
    description="RAG-powered API for answering documentation queries using Google Gemini embeddings",
    version="2.0.0"
)

# Configure CORS middleware to allow requests from Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Docusaurus default port
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# Validate required environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY")

if not all([QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY, ANTHROPIC_API_KEY]):
    missing_vars = []
    if not QDRANT_URL:
        missing_vars.append("QDRANT_URL")
    if not QDRANT_API_KEY:
        missing_vars.append("QDRANT_API_KEY")
    if not GEMINI_API_KEY:
        missing_vars.append("GEMINI_API_KEY")
    if not ANTHROPIC_API_KEY:
        missing_vars.append("ANTHROPIC_API_KEY")

    raise ValueError(
        f"Missing required environment variables: {', '.join(missing_vars)}. "
        "Please ensure they are set in your .env file."
    )

# Initialize clients
try:
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    genai.configure(api_key=GEMINI_API_KEY)
    anthropic_client = anthropic.Anthropic(api_key=ANTHROPIC_API_KEY)
    print("✓ All API clients initialized successfully")
    print("✓ Using Google Gemini for embeddings")
    print("✓ Using Anthropic Claude for response generation")
except Exception as e:
    raise RuntimeError(f"Failed to initialize API clients: {str(e)}")

# Pydantic model for chat query request
class ChatQuery(BaseModel):
    """Request model for chat endpoint"""
    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's question about the documentation"
    )

    @validator('query')
    def query_must_not_be_empty(cls, v):
        """Validate that query is not just whitespace"""
        if not v.strip():
            raise ValueError('Query cannot be empty or whitespace only')
        return v.strip()

# Pydantic model for chat response
class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    response: str = Field(
        ...,
        description="AI-generated answer based on documentation context"
    )
    sources_count: Optional[int] = Field(
        None,
        description="Number of documentation chunks used as context"
    )


@app.get("/")
async def root():
    """Root endpoint - API health check"""
    return {
        "message": "Documentation Chatbot API is running",
        "status": "healthy",
        "version": "2.0.0",
        "embeddings": "Google Gemini (embedding-001)",
        "llm": "Anthropic Claude"
    }


@app.get("/health")
async def health_check():
    """Health check endpoint to verify service availability"""
    try:
        # Test Qdrant connection
        collections = qdrant_client.get_collections()

        return {
            "status": "healthy",
            "qdrant": "connected",
            "collections": [c.name for c in collections.collections],
            "embeddings_provider": "Google Gemini",
            "llm_provider": "Anthropic Claude"
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Service health check failed: {str(e)}"
        )


@app.post("/chat", response_model=ChatResponse)
async def chat(query: ChatQuery):
    """
    Process user query and return AI-generated response based on documentation.

    This endpoint:
    1. Embeds the user's query using Google Gemini embeddings
    2. Performs semantic search in Qdrant to retrieve top-3 relevant chunks
    3. Constructs a RAG prompt with context and query
    4. Calls Anthropic Claude API to generate the response
    5. Returns the generated answer

    Args:
        query: ChatQuery object containing the user's question

    Returns:
        ChatResponse object with the AI-generated answer

    Raises:
        HTTPException: For various error scenarios (400, 500, 503)
    """
    try:
        # Step 1: Generate embedding for the user's query using Gemini
        try:
            result = genai.embed_content(
                model="models/embedding-001",
                content=query.query,
                task_type="retrieval_query"
            )
            query_vector = result['embedding']
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to generate query embedding with Gemini: {str(e)}"
            )

        # Step 2: Perform similarity search in Qdrant to retrieve top-3 chunks
        collection_name = "docusaurus_docs"
        try:
            search_results = qdrant_client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=3  # Retrieve exactly 3 most relevant chunks
            )
        except Exception as e:
            # Check if it's a collection not found error
            error_msg = str(e).lower()
            if "not found" in error_msg or "does not exist" in error_msg:
                raise HTTPException(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    detail=(
                        f"Collection '{collection_name}' not found. "
                        "Please run the ingestion script (python ingest.py) first to create "
                        "and populate the collection with Gemini embeddings."
                    )
                )
            else:
                raise HTTPException(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    detail=f"Failed to search vector database: {str(e)}"
                )

        # Step 3: Extract content from search results
        if not search_results:
            # No relevant documentation found
            context_chunks = []
        else:
            context_chunks = []
            for result in search_results:
                content = result.payload.get("content", "")
                source = result.payload.get("source", "Unknown source")
                context_chunks.append(f"[Source: {source}]\n{content}")

        # Step 4: Construct the RAG system prompt
        if context_chunks:
            context_text = "\n\n---\n\n".join(context_chunks)
            system_prompt = f"""You are a helpful documentation assistant. Your role is to answer questions based ONLY on the provided documentation context.

IMPORTANT INSTRUCTIONS:
1. Answer the user's question using ONLY the information provided in the context below
2. If the context doesn't contain enough information to answer the question, say "I don't have enough information in the documentation to answer that question"
3. Do not make up or infer information that isn't explicitly stated in the context
4. Be concise and direct in your answers
5. If relevant, you can mention which part of the documentation the information comes from

DOCUMENTATION CONTEXT:
{context_text}

USER QUESTION:
{query.query}

Please provide a helpful answer based on the documentation context above."""
        else:
            system_prompt = f"""You are a helpful documentation assistant.

The user asked: "{query.query}"

Unfortunately, I couldn't find any relevant information in the documentation to answer this question. Please let the user know that this topic may not be covered in the current documentation, or they might want to rephrase their question."""

        # Step 5: Call Anthropic Claude API to generate response
        try:
            message = anthropic_client.messages.create(
                model="claude-3-5-sonnet-20241022",  # Using Claude 3.5 Sonnet
                max_tokens=1024,
                messages=[
                    {
                        "role": "user",
                        "content": system_prompt
                    }
                ]
            )

            # Extract the text response from Claude's message
            ai_response = message.content[0].text

        except anthropic.APIError as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Anthropic API error: {str(e)}"
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to generate AI response: {str(e)}"
            )

        # Step 6: Return the response
        return ChatResponse(
            response=ai_response,
            sources_count=len(context_chunks)
        )

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Catch any unexpected errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred: {str(e)}"
        )


# Entry point for running the application
if __name__ == "__main__":
    import uvicorn

    # Run the FastAPI application with uvicorn
    uvicorn.run(
        "app:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Enable auto-reload during development
        log_level="info"
    )
