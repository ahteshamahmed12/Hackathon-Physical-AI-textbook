import os
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
import google.generativeai as genai
from sentence_transformers import SentenceTransformer

# 1. Setup
load_dotenv()
embedder = SentenceTransformer("all-MiniLM-L6-v2") # Local model (384 dims)
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel("gemini-2.5-flash")

app = FastAPI()

# 2. CORS (Allows Docusaurus to talk to the API)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,  # Cache preflight requests for 1 hour
)

class ChatQuery(BaseModel):
    query: str

# 3. Routes
@app.get("/")
async def health_check():
    return {"status": "running", "model": "all-MiniLM-L6-v2", "vector_dim": 384}

@app.post("/chat")
async def chat(query: ChatQuery):
    try:
        # A. Vectorize locally
        vector = embedder.encode(query.query).tolist()

        # B. Search Qdrant (using query_points for max compatibility)
        search_result = client.query_points(
            collection_name="AI_Book",
            query=vector,
            limit=3
        )
        
        # C. Context extraction
        context = "\n---\n".join([p.payload.get("content", "") for p in search_result.points])
        
        if not context:
            return {"response": "I couldn't find any relevant information in the documents.", "sources_count": 0}

        # D. Gemini Answer
        prompt = f"Use this context to answer: {query.query}\n\nContext:\n{context}"
        response = model.generate_content(prompt)
        
        return {
            "response": response.text if response.candidates else "The AI could not generate an answer.",
            "sources_count": len(search_result.points)
        }

    except Exception as e:
        print(f"Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

class TranslateQuery(BaseModel):
    content: str
    target_language: str

@app.post("/translate")
async def translate(query: TranslateQuery):
    try:
        # Construct the translation prompt
        prompt = f"Translate the following text to {query.target_language}:\n\n---\n\n{query.content}"
        
        # Call the Gemini API for translation
        response = model.generate_content(prompt)
        
        translated_text = response.text if response.candidates else "Translation not available."

        return {"translated_content": translated_text}

    except Exception as e:
        print(f"Translation Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)