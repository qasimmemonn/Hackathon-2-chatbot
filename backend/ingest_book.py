import os
import uuid
import logging
import requests
from bs4 import BeautifulSoup
from urllib.parse import urlparse
from dotenv import load_dotenv

from cohere_client import get_embeddings
from vector_store import create_collection, upsert_points

load_dotenv()

SITEMAP_URL = os.getenv("SITEMAP_URL")
DOCUSAURUS_DOMAIN = os.getenv("DOCUSAURUS_DOMAIN")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


def get_sitemap_urls(sitemap_url: str) -> list[str]:
    urls = []
    
    # Extract base domain from SITEMAP_URL
    parsed_sitemap = urlparse(sitemap_url)
    base_domain = f"{parsed_sitemap.scheme}://{parsed_sitemap.netloc}"
    
    logging.info(f"Sitemap base domain: {base_domain}")
    
    try:
        res = requests.get(sitemap_url, timeout=30)
        res.raise_for_status()
        soup = BeautifulSoup(res.content, "xml")

        for loc in soup.find_all("loc"):
            url = loc.text.strip()
            
            # Fix example.com URLs
            if "https://your-docusaurus-site.example.com" in url:
                url = url.replace("https://your-docusaurus-site.example.com", base_domain)
            
            # Simple filter to ensure we actuaully get docs pages (optional, but good)
            # Docusaurus usually puts pages under /docs/ or just root.
            # We'll just collect everything that matches the base domain now
            if base_domain in url:
                urls.append(url)
            else:
                logging.warning(f"Skipping external URL: {url}")

    except Exception as e:
        logging.error(f"Sitemap error: {e}")

    return urls


def get_page_content(url: str) -> str:
    try:
        res = requests.get(url, timeout=30)
        res.raise_for_status()
        soup = BeautifulSoup(res.content, "html.parser")

        main = soup.find("article") or soup.find("main") or soup.body
        if not main:
            return ""

        for tag in main.find_all(
            ["nav", "aside", "footer", "header", "form", "button"]
        ):
            tag.decompose()

        return main.get_text(separator=" ", strip=True)

    except Exception as e:
        logging.error(f"Page fetch error ({url}): {e}")
        return ""


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> list[str]:
    words = text.split()
    chunks = []

    for i in range(0, len(words), chunk_size - overlap):
        chunk = " ".join(words[i : i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)

    return chunks


def ingest_book_content():
    logging.info("Starting ingestion process")

    # ✅ Ensure Qdrant collection exists
    create_collection()

    urls = get_sitemap_urls(SITEMAP_URL)
    logging.info(f"Found {len(urls)} pages")

    for url in urls:
        logging.info(f"Ingesting: {url}")

        content = get_page_content(url)
        if not content:
            continue

        chunks = chunk_text(content)
        if not chunks:
            continue

        embeddings = get_embeddings(
            texts=chunks,
            input_type="search_document"
        )

        points = []
        for text, vector in zip(chunks, embeddings):
            points.append({
                "id": str(uuid.uuid4()),
                "vector": vector,
                "payload": {
                    "url": url,
                    "title": os.path.basename(urlparse(url).path) or url,
                    "text": text
                }
            })

        upsert_points(points)
        logging.info(f"Stored {len(points)} chunks")

    logging.info("Ingestion completed successfully")


if __name__ == "__main__":
    ingest_book_content()
