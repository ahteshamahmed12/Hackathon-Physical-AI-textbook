# check_qdrant.py
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))

# Check if collection exists and has points
collection_info = client.get_collection(collection_name="AI_Book")
print(f"Points in collection: {collection_info.points_count}")