import os
from datetime import timedelta, datetime
from dotenv import load_dotenv
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from jose import JWTError, jwt
from pydantic import BaseModel
from qdrant_client import QdrantClient
import google.generativeai as genai
from sentence_transformers import SentenceTransformer
from sqlalchemy.orm import Session

import crud, models, schemas
from database import SessionLocal, engine

models.Base.metadata.create_all(bind=engine)

# 1. Setup
load_dotenv()
embedder = SentenceTransformer("all-MiniLM-L6-v2") # Local model (384 dims)
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel("gemini-2.5-flash")

# Security settings
SECRET_KEY = os.getenv("SECRET_KEY", "a_very_secret_key")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

app = FastAPI()

# Dependency
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="signin")

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

def create_access_token(data: dict, expires_delta: timedelta | None = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

async def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
        token_data = schemas.TokenData(email=email)
    except JWTError:
        raise credentials_exception
    user = crud.get_user_by_email(db, email=token_data.email)
    if user is None:
        raise credentials_exception
    return user

async def get_current_active_user(current_user: schemas.User = Depends(get_current_user)):
    if not current_user.is_active:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user

# 3. Routes
@app.post("/signup", response_model=schemas.User)
def signup(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = crud.get_user_by_email(db, email=user.email)
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    return crud.create_user(db=db, user=user)

@app.post("/signin", response_model=schemas.Token)
async def signin(form_data: OAuth2PasswordRequestForm = Depends(), db: Session = Depends(get_db)):
    user = crud.get_user_by_email(db, email=form_data.username)
    if not user or not crud.pwd_context.verify(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@app.get("/users/me/", response_model=schemas.User)
async def read_users_me(current_user: schemas.User = Depends(get_current_active_user)):
    return current_user

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

class SearchQuery(BaseModel):
    query: str

@app.post("/search")
async def search(query: SearchQuery):
    try:
        # A. Vectorize locally
        vector = embedder.encode(query.query).tolist()

        # B. Search Qdrant
        search_result = client.search(
            collection_name="AI_Book",
            query_vector=vector,
            limit=5,
            with_payload=True,
        )
        
        # C. Format results
        results = []
        for point in search_result:
            results.append(
                {
                    "score": point.score,
                    "content": point.payload.get("content", ""),
                    "metadata": point.payload.get("metadata", {}),
                }
            )
        
        return {"results": results}

    except Exception as e:
        print(f"Search Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)