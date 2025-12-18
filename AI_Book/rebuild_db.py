import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

load_dotenv()

# Initialize Qdrant Client
client = QdrantClient(
    url=os.getenv("QDRANT_URL"), 
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "AI_Book"

print(f"🔄 Resetting collection: {COLLECTION_NAME}...")

# 1. Delete the old mismatched collection
try:
    client.delete_collection(collection_name=COLLECTION_NAME)
    print("✅ Deleted old 1536-dim collection.")
except Exception as e:
    print(f"⚠️ Collection might not exist yet: {e}")

# 2. Create the new 384-dim collection
client.create_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=VectorParams(size=384, distance=Distance.COSINE)
)
print("✅ Created new 384-dim collection. Ready for ingestion!")