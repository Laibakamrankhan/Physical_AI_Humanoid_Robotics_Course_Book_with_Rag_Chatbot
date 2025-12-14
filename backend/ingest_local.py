import os
import logging
from typing import List, Dict, Any
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
import time
import uuid
import re
from pathlib import Path

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LocalMarkdownIngestion:
    def __init__(self):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url)

    def extract_text_from_markdown(self, file_path: str) -> str:
        """
        Extract text content from a markdown file
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

                # Remove markdown formatting like headers, links, etc. to get clean text
                # Remove markdown headers
                content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
                # Remove markdown links [text](url)
                content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)
                # Remove markdown bold/italic
                content = re.sub(r'\*{1,2}([^*]+)\*{1,2}', r'\1', content)
                # Remove markdown code blocks
                content = re.sub(r'```(?:\w+\n)?(.*?)```', r'\1', content, flags=re.DOTALL)
                # Remove inline code
                content = re.sub(r'`([^`]+)`', r'\1', content)

                return content.strip()
        except Exception as e:
            logger.error(f"Error reading markdown file {file_path}: {e}")
            return ""

    def chunk_text(self, text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
        """
        Split text into chunks with overlap to preserve context
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunk = text[start:end]
            chunks.append(chunk)

            # Move start position by chunk_size - overlap
            start = end - overlap

            # If remaining text is less than chunk_size, add it as final chunk
            if len(text) - start < chunk_size:
                if start < len(text):
                    final_chunk = text[start:]
                    if final_chunk not in chunks:  # Avoid duplicate chunks
                        chunks.append(final_chunk)
                break

        return chunks

    def embed(self, text: str) -> List[float]:
        """
        Generate embedding for text using Cohere
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-multilingual-v3.0",  # Using multilingual model
                input_type="search_document"  # Optimize for search
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            logger.error(f"Error generating embedding for text: {e}")
            return []

    def create_collection(self, collection_name: str = "rag_embedding"):
        """
        Create a Qdrant collection for storing embeddings
        """
        try:
            # Check if collection already exists
            collections = self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if collection_name in collection_names:
                logger.info(f"Collection {collection_name} already exists")
                return

            # Create collection with appropriate vector size (1024 for Cohere embeddings)
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )

            logger.info(f"Created collection {collection_name} with 1024-dimension vectors")

        except Exception as e:
            logger.error(f"Error creating collection {collection_name}: {e}")
            raise

    def save_chunk_to_qdrant(self, content: str, url: str, embedding: List[float], position: int, collection_name: str = "rag_embedding"):
        """
        Save a text chunk with its embedding to Qdrant
        """
        try:
            # Generate a unique ID for the point
            point_id = str(uuid.uuid4())

            # Prepare the payload with metadata
            payload = {
                "content": content,
                "url": url,
                "position": position,
                "created_at": time.time()
            }

            # Create and upload the point to Qdrant
            points = [PointStruct(
                id=point_id,
                vector=embedding,
                payload=payload
            )]

            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=points
            )

            logger.info(f"Saved chunk to Qdrant: {url} (position {position})")
            return True

        except Exception as e:
            logger.error(f"Error saving chunk to Qdrant: {e}")
            return False

def main():
    """
    Main function to ingest local markdown files into the RAG system
    """
    logger.info("Starting Local Markdown Ingestion Pipeline...")

    # Initialize the ingestion pipeline
    ingestion = LocalMarkdownIngestion()

    try:
        # Step 1: Create the Qdrant collection
        logger.info("Creating Qdrant collection...")
        ingestion.create_collection("rag_embedding")

        # Step 2: Find all markdown files in the book directory
        book_dir = Path("1-physical-ai-robotics")
        markdown_files = list(book_dir.rglob("*.md"))

        if not markdown_files:
            logger.warning(f"No markdown files found in {book_dir}")
            return

        logger.info(f"Found {len(markdown_files)} markdown files to process")

        # Step 3: Process each markdown file
        total_chunks = 0
        for i, file_path in enumerate(markdown_files):
            logger.info(f"Processing file {i+1}/{len(markdown_files)}: {file_path}")

            # Extract text from the markdown file
            text_content = ingestion.extract_text_from_markdown(str(file_path))

            if not text_content:
                logger.warning(f"No content extracted from {file_path}")
                continue

            logger.info(f"Extracted {len(text_content)} characters from {file_path}")

            # Use the relative file path as the URL for reference
            relative_path = str(file_path.relative_to(Path(".")))
            url_reference = f"file://{relative_path}"

            # Chunk the text
            chunks = ingestion.chunk_text(text_content)
            logger.info(f"Created {len(chunks)} chunks from {file_path}")

            # Process each chunk
            for j, chunk in enumerate(chunks):
                if not chunk.strip():
                    continue

                # Generate embedding
                embedding = ingestion.embed(chunk)

                if not embedding:
                    logger.error(f"Failed to generate embedding for chunk {j} of {file_path}")
                    continue

                # Save to Qdrant
                success = ingestion.save_chunk_to_qdrant(
                    content=chunk,
                    url=url_reference,
                    embedding=embedding,
                    position=j
                )

                if success:
                    total_chunks += 1
                    logger.info(f"Successfully saved chunk {j} of {file_path} to Qdrant")
                else:
                    logger.error(f"Failed to save chunk {j} of {file_path} to Qdrant")

        logger.info(f"Ingestion pipeline completed successfully! Total chunks saved: {total_chunks}")

    except Exception as e:
        logger.error(f"Ingestion pipeline failed with error: {e}")
        raise

if __name__ == "__main__":
    main()