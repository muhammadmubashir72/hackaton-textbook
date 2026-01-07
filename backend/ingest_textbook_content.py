"""
Script to ingest the actual textbook content into Qdrant using Cohere embeddings
This focuses on the main textbook content files rather than just technical specs
"""
import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
import os
from dotenv import load_dotenv
import glob
import markdown
from pathlib import Path

# Load environment variables
load_dotenv()

# -------------------------------------
# CONFIG
# -------------------------------------
SITEMAP_URL = os.getenv("SITEMAP_URL", "https://hackaton-ai-textbook.vercel.app/sitemap.xml")
ROOT_DIR = os.getenv("ROOT_DIR", "..")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "humanoid_ai_book")

cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
EMBED_MODEL = os.getenv("EMBEDDING_MODEL", "embed-english-v3.0")

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# -------------------------------------
# Step 1 — Extract local textbook markdown files from root and subdirectories
# -------------------------------------
def get_textbook_docs(root_dir):
    """Get all important markdown files that contain textbook content"""
    
    # Define the important textbook content files
    textbook_files = [
        os.path.join(root_dir, "Hackathon.md"),
        os.path.join(root_dir, "README.md"),
        os.path.join(root_dir, "CLAUDE.md"),
        os.path.join(root_dir, "prompt.md"),
    ]
    
    # Also add any markdown files from the frontend docs
    frontend_docs = glob.glob(os.path.join(root_dir, "frontend", "**", "*.md"), recursive=True)
    all_files = textbook_files + frontend_docs
    
    # Filter to only include files that actually exist
    existing_files = []
    for file_path in all_files:
        if os.path.exists(file_path):
            existing_files.append(file_path)
            print(f" - Found textbook content: {file_path}")
        else:
            print(f" - Missing: {file_path}")
    
    # Also include markdown files from specs directory if they contain content
    specs_files = glob.glob(os.path.join(root_dir, "specs", "**", "*.md"), recursive=True)
    for file_path in specs_files:
        if os.path.exists(file_path):
            existing_files.append(file_path)
            print(f" - Found spec content: {file_path}")
    
    return existing_files


# -------------------------------------
# Step 2 — Read content from local markdown files
# -------------------------------------
def read_local_file(file_path):
    """Read and extract text content from local markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # For markdown files, convert to plain text
        try:
            # Convert markdown to HTML then extract text
            html = markdown.markdown(content)
            text = trafilatura.extract(html) if html else content
        except:
            # If markdown conversion fails, just return the content
            text = content

        return text
    except Exception as e:
        print(f"[ERROR] Could not read file {file_path}: {e}")
        return None


# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    """Split text into smaller chunks while trying to preserve sentence boundaries"""
    chunks = []
    while len(text) > max_chars:
        # Try to find a good break point
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = text[:max_chars].rfind("! ")
        if split_pos == -1:
            split_pos = text[:max_chars].rfind("? ")
        if split_pos == -1:
            split_pos = text[:max_chars].rfind("\n")
        if split_pos == -1:
            split_pos = max_chars  # Last resort, break mid-sentence
        
        if split_pos == max_chars:
            chunks.append(text[:split_pos])
            text = text[split_pos:]
        else:
            chunks.append(text[:split_pos+1])
            text = text[split_pos+1:]
    
    if text:  # Add the remaining text if any
        chunks.append(text)
    
    return chunks


# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    try:
        response = cohere_client.embed(
            model=EMBED_MODEL,
            input_type="search_document",  # Use search_document for ingestion
            texts=[text],
        )
        return response.embeddings[0]  # Return the first embedding
    except Exception as e:
        print(f"[ERROR] Could not create embedding: {e}")
        return None


# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,        # Cohere embed-english-v3.0 dimension
            distance=Distance.COSINE
        )
    )


def save_chunk_to_qdrant(chunk, chunk_id, source):
    vector = embed(chunk)
    if vector is None:
        print(f"[ERROR] Could not embed chunk {chunk_id}, skipping")
        return False

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "source": source,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )
    return True


# -------------------------------------
# MAIN TEXTBOOK INGESTION PIPELINE
# -------------------------------------
def ingest_textbook_documents():
    # Get local textbook markdown files
    local_files = get_textbook_docs(ROOT_DIR)

    create_collection()

    global_id = 1

    # Process local markdown files that contain actual textbook content
    for file_path in local_files:
        print(f"\nProcessing textbook content file: {file_path}")
        text = read_local_file(file_path)

        if not text:
            print(f"[ERROR] Could not read content from {file_path}")
            continue

        print(f"Content length: {len(text)} characters")
        
        # Skip very small files
        if len(text) < 50:
            print(f"Skipping very small file: {file_path}")
            continue

        chunks = chunk_text(text)

        print(f"Created {len(chunks)} chunks from {file_path}")
        for i, ch in enumerate(chunks):
            if len(ch.strip()) > 20:  # Only save chunks with meaningful content
                success = save_chunk_to_qdrant(ch, global_id, file_path)
                if success:
                    print(f"Saved chunk {global_id} from file: {file_path}")
                    global_id += 1
            else:
                print(f"Skipping small chunk {i} from {file_path}")

    print(f"\n✔️ Textbook content ingestion completed!")
    print(f"Total chunks stored: {global_id - 1}")
    print(f"Collection: {COLLECTION_NAME}")


if __name__ == "__main__":
    print("Starting textbook content ingestion...")
    print("This will index the actual textbook content including Hackathon.md and other educational materials")
    ingest_textbook_documents()