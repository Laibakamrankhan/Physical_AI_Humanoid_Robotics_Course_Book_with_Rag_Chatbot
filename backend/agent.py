import os
import json
import logging
from typing import Dict, List
from dotenv import load_dotenv
import asyncio
import time
import re

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def retrieve_information(query: str) -> Dict:
    """
    Retrieve information from the knowledge base based on a query
    """
    from retriving import RAGRetriever
    retriever = RAGRetriever()

    try:
        # Call the existing retrieve method from the RAGRetriever instance
        json_response = retriever.retrieve(query_text=query, top_k=5, threshold=0.1)
        results = json.loads(json_response)

        # Format the results for the assistant
        formatted_results = []
        for result in results.get('results', []):
            formatted_results.append({
                'content': result['content'],
                'url': result['url'],
                'position': result['position'],
                'similarity_score': result['similarity_score']
            })

        return {
            'query': query,
            'retrieved_chunks': formatted_results,
            'total_results': len(formatted_results)
        }
    except Exception as e:
        logger.error(f"Error in retrieve_information: {e}")
        return {
            'query': query,
            'retrieved_chunks': [],
            'total_results': 0,
            'error': str(e)
        }

class RAGAgent:
    def __init__(self):
        logger.info("RAG Agent initialized without OpenAI API (using local retrieval and rule-based responses)")

    def query_agent(self, query_text: str) -> Dict:
        """
        Process a query through the RAG agent and return structured response
        """
        start_time = time.time()
        logger.info(f"Processing query through RAG agent: '{query_text[:50]}...'")

        try:
            # Retrieve relevant information first
            retrieval_result = retrieve_information(query_text)
            retrieved_chunks = retrieval_result.get('retrieved_chunks', [])

            # Format the context from retrieved information
            context_str = "\n\n".join([f"Source: {chunk['url']}\nContent: {chunk['content']}"
                                     for chunk in retrieved_chunks])

            # Generate a response based on the retrieved context
           if context_str:
    # Extract key info from top chunk only
    top_chunk = retrieved_chunks[0] if retrieved_chunks else None
    if top_chunk:
        content = top_chunk['content']
        clean_content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
        clean_content = re.sub(r'\n\d+\.\d*\.?\s+', '\n', clean_content)
        clean_content = re.sub(r'\n\s*-\s+', '\n• ', clean_content)

        sentences = [s.strip() for s in clean_content.split('.') if len(s.strip()) > 10]
        query_words = query_text.lower().split()[:3]

        # Pick the first relevant sentence containing any query word
        answer_sentence = next((s for s in sentences if any(q in s.lower() for q in query_words)), sentences[0] if sentences else "")
        if len(answer_sentence) > 200:
            answer_sentence = answer_sentence[:200] + "..."

        answer = f"Based on the book content: {answer_sentence}"
    else:
        answer = "I couldn't find relevant information in the knowledge base."

            sources = list(set([chunk['url'] for chunk in retrieved_chunks if chunk['url']]))
            query_time_ms = (time.time() - start_time) * 1000

            return {
                "answer": answer,
                "sources": sources,
                "matched_chunks": retrieved_chunks,
                "query_time_ms": query_time_ms,
                "confidence": self._calculate_confidence(retrieved_chunks)
            }

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            return {
                "answer": "Sorry, I encountered an error processing your request.",
                "sources": [],
                "matched_chunks": [],
                "error": str(e),
                "query_time_ms": (time.time() - start_time) * 1000
            }

    def _calculate_confidence(self, matched_chunks: List[Dict]) -> str:
        if not matched_chunks:
            return "low"
        avg_score = sum(chunk.get('similarity_score', 0.0) for chunk in matched_chunks) / len(matched_chunks)
        if avg_score >= 0.7:
            return "high"
        elif avg_score >= 0.4:
            return "medium"
        else:
            return "low"

def query_agent(query_text: str) -> Dict:
    agent = RAGAgent()
    return agent.query_agent(query_text)

def main():
    logger.info("Initializing RAG Agent...")

    agent = RAGAgent()
    test_queries = [
        "what is a digital twin in robotics?",
    ]

    print("RAG Agent - Testing Queries")
    print("=" * 50)

    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        print("-" * 30)
        response = agent.query_agent(query)
        print(f"Answer: {response['answer']}")
        if response.get('sources'):
            print(f"Sources: {len(response['sources'])} documents")
            for source in response['sources'][:3]:
                print(f"  - {source}")
        if response.get('matched_chunks'):
            print(f"Matched chunks: {len(response['matched_chunks'])}")
            for j, chunk in enumerate(response['matched_chunks'][:2], 1):
                content_preview = chunk['content'][:100] + "..." if len(chunk['content']) > 100 else chunk['content']
                print(f"  Chunk {j}: {content_preview}")
                print(f"    Source: {chunk['url']}")
                print(f"    Score: {chunk['similarity_score']:.3f}")
        print(f"Query time: {response['query_time_ms']:.2f}ms")
        print(f"Confidence: {response.get('confidence', 'unknown')}")
        if i < len(test_queries):
            time.sleep(1)

if __name__ == "__main__":
    main()
