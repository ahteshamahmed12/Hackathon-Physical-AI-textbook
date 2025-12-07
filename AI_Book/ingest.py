"""
Document Ingestion Script for RAG System with Google Gemini Embeddings

This script processes documentation files and creates vector embeddings using
Google's Gemini API, storing them in Qdrant vector database.

Requirements:
- Google Gemini API key
- Qdrant vector database credentials
- Documentation files in ./docs directory
"""

import os
import glob
from dotenv import load_dotenv
from langchain_community.document_loaders import TextLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from qdrant_client import QdrantClient, models
import google.generativeai as genai

# 1. Load environment variables from .env
load_dotenv()

# 2. Get environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not all([QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY]):
    raise ValueError(
        "Missing one or more environment variables: QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY. "
        "Please ensure they are set in your .env file."
    )

# 3. Configure Gemini API
genai.configure(api_key=GEMINI_API_KEY)

print("✓ Environment variables loaded successfully")
print(f"✓ Using Gemini API for embeddings")

# 4. Use glob to find all markdown files (.md, .mdx) recursively in the ./docs directory
docs_path = "./docs"
markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
markdown_files.extend(glob.glob(f"{docs_path}/**/*.mdx", recursive=True))

print(f"\n📁 Scanning {docs_path} directory...")
print(f"✓ Found {len(markdown_files)} markdown files")

# 5. Load documents
documents = []
for file_path in markdown_files:
    try:
        loader = TextLoader(file_path, encoding='utf-8')
        doc = loader.load()[0]  # Load returns a list, we take the first document
        doc.metadata["source"] = file_path  # Add source filename to metadata
        documents.append(doc)
        print(f"  ✓ Loaded: {file_path}")
    except Exception as e:
        print(f"  ✗ Error loading {file_path}: {e}")

if not documents:
    print(f"\n❌ No markdown files found in {docs_path}. Exiting.")
    exit(1)

print(f"\n✓ Successfully loaded {len(documents)} documents")

# 6. Use RecursiveCharacterTextSplitter to chunk the documents
print(f"\n✂️  Splitting documents into chunks (size=1000, overlap=200)...")
text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
chunks = text_splitter.split_documents(documents)
print(f"✓ Created {len(chunks)} chunks")

# 7. Initialize the QdrantClient
print(f"\n🔗 Connecting to Qdrant...")
try:
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    print(f"✓ Connected to Qdrant at {QDRANT_URL}")
except Exception as e:
    print(f"❌ Failed to connect to Qdrant: {e}")
    exit(1)

# 8. Create a collection named docusaurus_docs with the correct vector size
collection_name = "docusaurus_docs"
# Gemini embedding-001 model produces 768-dimensional vectors
vector_size = 768

print(f"\n📦 Setting up collection '{collection_name}'...")

# Check if collection already exists
try:
    existing_collection = client.get_collection(collection_name=collection_name)
    print(f"⚠️  Collection '{collection_name}' already exists.")

    # Check if vector size matches
    if existing_collection.config.params.vectors.size != vector_size:
        print(f"⚠️  Existing collection has vector size {existing_collection.config.params.vectors.size}, but Gemini requires {vector_size}")
        print(f"🔄 Recreating collection with correct vector size...")
        client.delete_collection(collection_name=collection_name)
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        print(f"✓ Collection '{collection_name}' recreated successfully")
    else:
        print(f"✓ Using existing collection (vector size: {vector_size})")
except Exception:
    print(f"📝 Collection '{collection_name}' does not exist. Creating it now...")
    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        print(f"✓ Collection '{collection_name}' created successfully")
    except Exception as e:
        print(f"❌ Failed to create collection: {e}")
        exit(1)

# 9. Ingest the document chunks into the Qdrant collection
print(f"\n🚀 Ingesting {len(chunks)} chunks into collection '{collection_name}'...")

# Prepare data for Qdrant
points = []
failed_chunks = 0

for i, chunk in enumerate(chunks):
    # Progress indicator
    if (i + 1) % 10 == 0 or i == 0:
        print(f"  Processing chunk {i + 1}/{len(chunks)}...")

    # Ensure chunk.page_content is a string
    if not isinstance(chunk.page_content, str):
        print(f"  ⚠️  Skipping chunk {i} due to non-string content: {type(chunk.page_content)}")
        failed_chunks += 1
        continue

    # Generate embedding for the chunk using Gemini
    try:
        # Use Gemini's embedding model
        result = genai.embed_content(
            model="models/embedding-001",
            content=chunk.page_content,
            task_type="retrieval_document"
        )
        vector = result['embedding']

    except Exception as e:
        print(f"  ✗ Error generating embedding for chunk {i}: {e}. Skipping this chunk.")
        failed_chunks += 1
        continue

    # Use a unique ID for each point
    point_id = i

    # Payload includes original metadata and page_content
    payload = chunk.metadata.copy()
    payload["content"] = chunk.page_content

    points.append(
        models.PointStruct(
            id=point_id,
            vector=vector,
            payload=payload,
        )
    )

print(f"\n📊 Ingestion Summary:")
print(f"  Total chunks: {len(chunks)}")
print(f"  Successfully processed: {len(points)}")
print(f"  Failed: {failed_chunks}")

if points:
    print(f"\n💾 Uploading {len(points)} vectors to Qdrant...")
    try:
        client.upsert(
            collection_name=collection_name,
            wait=True,
            points=points
        )
        print(f"✓ Successfully ingested {len(points)} chunks into '{collection_name}'")
    except Exception as e:
        print(f"❌ Failed to upload vectors: {e}")
        exit(1)
else:
    print("❌ No valid chunks to ingest.")
    exit(1)

print(f"\n✅ Ingestion completed successfully!")
print(f"📌 Collection '{collection_name}' is ready for queries")
print(f"🔢 Vector size: {vector_size} (Gemini embedding-001)")
