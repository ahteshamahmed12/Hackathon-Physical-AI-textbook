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
# For development, we're permissive. Tighten this for production.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development (tighten for production)
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
    expose_headers=["*"],
    max_age=3600,  # Cache preflight requests for 1 hour
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
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        prefer_grpc=False,  # Use REST API instead of gRPC for better compatibility
    )
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
                model="claude-3-haiku-20240307",  # Using Claude 3 Haiku (most compatible)
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


# Pydantic model for translation request
class TranslationRequest(BaseModel):
    """Request model for translation endpoint"""
    content: str = Field(
        ...,
        min_length=1,
        max_length=50000,
        description="Markdown content to translate to Urdu"
    )
    target_language: str = Field(
        default="Urdu",
        description="Target language for translation"
    )

    @validator('content')
    def content_must_not_be_empty(cls, v):
        """Validate that content is not just whitespace"""
        if not v.strip():
            raise ValueError('Content cannot be empty or whitespace only')
        return v.strip()


# Pydantic model for translation response
class TranslationResponse(BaseModel):
    """Response model for translation endpoint"""
    translated_content: str = Field(
        ...,
        description="Translated markdown content"
    )
    original_length: int = Field(
        ...,
        description="Character count of original content"
    )
    translated_length: int = Field(
        ...,
        description="Character count of translated content"
    )


@app.post("/translate", response_model=TranslationResponse)
async def translate_content(request: TranslationRequest):
    """
    Translate markdown documentation content to Urdu using Claude API.

    This endpoint:
    1. Receives markdown content in English
    2. Sends it to Claude API with translation instructions
    3. Returns the translated content in Urdu while preserving markdown formatting

    Args:
        request: TranslationRequest object containing content to translate

    Returns:
        TranslationResponse object with translated content

    Raises:
        HTTPException: For various error scenarios (400, 500, 503)
    """
    try:
        # Construct translation prompt
        translation_prompt = f"""You are an expert translator specializing in technical documentation.
Your task is to translate the following markdown documentation from English to Urdu.

IMPORTANT INSTRUCTIONS:
1. Translate ALL text content to Urdu, including headings, paragraphs, list items, and table content
2. PRESERVE all markdown formatting (headers #, lists -, code blocks ```, links, bold, italic, etc.)
3. DO NOT translate:
   - Code snippets within backticks or code blocks
   - URLs and links
   - Technical commands and code syntax
   - File paths and system commands
4. Maintain the document structure exactly as it appears
5. Use clear, technical Urdu that would be understood by software engineers and robotics students
6. Keep technical terms in English when no good Urdu equivalent exists, but explain them in Urdu
7. Ensure the translation is accurate, natural-sounding, and professionally written

CONTENT TO TRANSLATE:

{request.content}

Please provide ONLY the translated markdown content, without any explanations or additional text."""

        # Call Claude API for translation
        try:
            message = anthropic_client.messages.create(
                model="claude-3-haiku-20240307",  # Using Claude 3 Haiku (most compatible)
                max_tokens=4096,  # Maximum for Claude 3 Haiku
                messages=[
                    {
                        "role": "user",
                        "content": translation_prompt
                    }
                ]
            )

            # Extract translated text
            translated_content = message.content[0].text

        except anthropic.APIError as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Claude API error during translation: {str(e)}"
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to generate translation: {str(e)}"
            )

        # Return translation response
        return TranslationResponse(
            translated_content=translated_content,
            original_length=len(request.content),
            translated_length=len(translated_content)
        )

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Catch any unexpected errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred during translation: {str(e)}"
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
