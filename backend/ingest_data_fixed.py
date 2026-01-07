"""
Enhanced script to ingest data from sitemap or local files into Qdrant using Cohere embeddings
Supports both web scraping from sitemap and reading local documents
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
LOCAL_DOCS_DIR = os.getenv("LOCAL_DOCS_DIR", "./specs")
COLLECTION_NAME = os.getenv("DOCUMENT_COLLECTION_NAME", "humanoid_ai_book") or os.getenv("COLLECTION_NAME", "humanoid_ai_book")

cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
EMBED_MODEL = os.getenv("EMBEDDING_MODEL", "embed-english-v3.0")

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    try:
        headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'}
        xml = requests.get(sitemap_url, headers=headers).text
        root = ET.fromstring(xml)

        urls = []
        for child in root:
            loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
            if loc_tag is not None:
                urls.append(loc_tag.text)

        print("\nFOUND URLS FROM SITEMAP:")
        for u in urls:
            print(" -", u)

        return urls
    except Exception as e:
        print(f"\n[WARNING] Could not fetch sitemap: {e}")
        return []


# -------------------------------------
# Step 2 — Extract local markdown files
# -------------------------------------
def get_local_docs(docs_dir):
    """Get all markdown files from local directory"""
    if not os.path.exists(docs_dir):
        print(f"\n[WARNING] Local docs directory {docs_dir} does not exist")
        return []

    md_files = glob.glob(os.path.join(docs_dir, "**/*.md"), recursive=True)
    md_files.extend(glob.glob(os.path.join(docs_dir, "**/*.mdx"), recursive=True))

    print(f"\nFOUND LOCAL MARKDOWN FILES:")
    for file in md_files:
        print(" -", file)

    return md_files


# -------------------------------------
# Step 3 — Read content from local markdown files
# -------------------------------------
def read_local_file(file_path):
    """Read and extract text content from local markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # For markdown files, we'll use markdown library to convert to plain text
        # Alternatively, you can use trafilatura or just return the raw content
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
# Step 4 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    try:
        headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'}
        html = requests.get(url, headers=headers).text
        text = trafilatura.extract(html)

        if not text:
            print(f"[WARNING] No text extracted from: {url}")
            # Try alternative approach for sites that might return content differently
            # Look for main content areas in the HTML
            import re
            # Find content in divs with common content class names
            content_patterns = [
                r'<article[^>]*>(.*?)</article>',
                r'<main[^>]*>(.*?)</main>',
                r'<div[^>]*class=["\'][^"\']*content[^"\']*["\'][^>]*>(.*?)</div>',
                r'<div[^>]*id=["\'][^"\']*content[^"\']*["\'][^>]*>(.*?)</div>',
                r'<section[^>]*>(.*?)</section>'
            ]
            for pattern in content_patterns:
                matches = re.findall(pattern, html, re.DOTALL | re.IGNORECASE)
                if matches:
                    text = ' '.join(matches)
                    # Clean up HTML tags
                    text = re.sub(r'<[^>]+>', '', text)
                    break

        return text
    except Exception as e:
        print(f"[ERROR] Could not fetch URL {url}: {e}")
        return None


# -------------------------------------
# Step 5 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        # Find a sentence end or paragraph break near the max_chars limit
        search_start = max_chars - 200 if max_chars > 200 else 0
        split_pos = -1
        
        # First try to find a period followed by whitespace
        for i in range(len(text) - 1, search_start, -1):
            if text[i] in ['.', '!', '?'] and i < len(text) - 1 and text[i+1].isspace():
                split_pos = i + 1
                break
        
        # If no sentence ending found, try to break at paragraph or new line
        if split_pos == -1:
            for i in range(max_chars, search_start, -1):
                if text[i] in ['\n', '\r']:
                    split_pos = i
                    break
        
        # If still no good break point, just cut at max_chars
        if split_pos == -1:
            split_pos = max_chars
            
        chunks.append(text[:split_pos])
        text = text[split_pos:].strip()
    
    if text.strip():  # Add remaining text if there is any
        chunks.append(text)
    
    return chunks


# -------------------------------------
# Step 6 — Create embedding
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
# Step 7 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    try:
        # Check if collection already exists and delete it if it does
        qdrant.get_collection(COLLECTION_NAME)
        print(f"Collection {COLLECTION_NAME} already exists, dropping it...")
        qdrant.delete_collection(COLLECTION_NAME)
    except:
        print(f"Collection {COLLECTION_NAME} does not exist yet, creating new one...")
    
    qdrant.create_collection(
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
# MAIN ENHANCED INGESTION PIPELINE
# -------------------------------------
def ingest_documents():
    # Get URLs from sitemap
    urls = get_all_urls(SITEMAP_URL)

    # Get local markdown files
    local_files = get_local_docs(LOCAL_DOCS_DIR)

    create_collection()

    global_id = 1

    # Process URLs from sitemap
    for url in urls[:5]:  # Limit to first 5 URLs to test initially
        print(f"\nProcessing URL: {url}")
        text = extract_text_from_url(url)

        if not text:
            print(f"Skipping URL (no content extracted): {url}")
            continue

        print(f"Extracted {len(text)} characters from {url}")
        chunks = chunk_text(text)

        for i, ch in enumerate(chunks):
            print(f"Saving chunk {global_id} ({len(ch)} chars) from {url}")
            success = save_chunk_to_qdrant(ch, global_id, url)
            if success:
                global_id += 1
            else:
                print(f"Failed to save chunk {global_id}")

    # Process local markdown files if they exist
    for file_path in local_files:
        print(f"\nProcessing local file: {file_path}")
        text = read_local_file(file_path)

        if not text:
            continue

        chunks = chunk_text(text)

        for i, ch in enumerate(chunks):
            success = save_chunk_to_qdrant(ch, global_id, file_path)
            if success:
                print(f"Saved chunk {global_id} from file: {file_path}")
                global_id += 1

    print(f"\nIngestion completed! Total chunks stored: {global_id - 1}")


if __name__ == "__main__":
    ingest_documents()