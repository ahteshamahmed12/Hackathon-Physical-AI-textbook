import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"), 
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "AI_Book"

# 1. Wipe the old, incorrectly-sized collection
print(f"Deleting collection: {COLLECTION_NAME}...")
try:
    client.delete_collection(collection_name=COLLECTION_NAME)
    print("✓ Deleted.")
except Exception as e:
    print(f"Note: {e}")

# 2. Recreate with 384 dimensions to match all-MiniLM-L6-v2
print(f"Creating collection with 384 dimensions...")
client.create_collection(
    collection_name=COLLECTION_NAME,
    vectors_config=VectorParams(size=384, distance=Distance.COSINE)
)

print("✅ Collection is now ready for 384-dim vectors!")