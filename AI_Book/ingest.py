import os
import glob
from dotenv import load_dotenv
from langchain_community.document_loaders import TextLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from qdrant_client import QdrantClient, models
from sentence_transformers import SentenceTransformer

load_dotenv()

# 1. Use Local Embedder (No API Key needed for this part!)
# This model is small, fast, and runs on your CPU.
embedder = SentenceTransformer("all-MiniLM-L6-v2")
VECTOR_SIZE = 384 

# 2. Setup Qdrant
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
COLLECTION_NAME = "AI_Book"

# 3. Load and Split Docs (Same logic as yours)
markdown_files = glob.glob("./docs/**/*.md", recursive=True)
documents = []
for file_path in markdown_files:
    loader = TextLoader(file_path, encoding='utf-8')
    doc = loader.load()[0]
    doc.metadata["source"] = file_path
    documents.append(doc)

text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
chunks = text_splitter.split_documents(documents)

# 4. Recreate Collection for the new Vector Size (384)
client.recreate_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
)

# 5. Local Ingestion (Unlimited & Fast)
points = []
for i, chunk in enumerate(chunks):
    # This happens LOCALLY on your machine
    vector = embedder.encode(chunk.page_content).tolist()
    
    payload = chunk.metadata.copy()
    payload["content"] = chunk.page_content

    points.append(models.PointStruct(id=i, vector=vector, payload=payload))

client.upsert(collection_name=COLLECTION_NAME, points=points)
print(f"✅ Successfully ingested {len(points)} chunks locally!")