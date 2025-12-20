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
                # Extract key information from top chunks
                relevant_chunks = retrieved_chunks[:2]
                concise_parts = []

                for chunk in relevant_chunks:
                    content = chunk['content']
                    clean_content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
                    clean_content = re.sub(r'\n\d+\.\d*\.?\s+', '\n', clean_content)
                    clean_content = re.sub(r'\n\s*-\s+', '\n• ', clean_content)

                    sentences = clean_content.split('.')
                    query_lower = query_text.lower()
                    relevant_sentences = []

                    for sentence in sentences:
                        clean_sentence = sentence.strip()
                        if len(clean_sentence) < 10:
                            continue
                        if any(header in clean_sentence for header in ['#', '##', '###', 'Chapter', 'Section', 'Module']):
                            continue
                        if any(term in clean_sentence.lower() for term in query_lower.split()[:3]):
                            relevant_sentences.append(clean_sentence)

                    if relevant_sentences:
                        selected = '. '.join(relevant_sentences[:2]) + '.'
                        if len(selected) > 250:
                            selected = selected[:250] + "..."
                        concise_parts.append(selected.strip())

                if concise_parts:
                    answer = "Based on the book content: " + " ".join(concise_parts[:2])
                else:
                    # Fallback: use first meaningful paragraph from top chunk
                    most_relevant = retrieved_chunks[0]['content'] if retrieved_chunks else ""
                    clean_content = re.sub(r'^#+\s+', '', most_relevant, flags=re.MULTILINE)
                    clean_content = re.sub(r'\n\d+\.\d*\.?\s+', '\n', clean_content)
                    paragraphs = [p.strip() for p in clean_content.split('\n') if p.strip()]
                    for paragraph in paragraphs:
                        if len(paragraph) > 50 and not any(header in paragraph for header in ['#', '##', '###', 'Chapter', 'Section', 'Module']):
                            if len(paragraph) > 300:
                                paragraph = paragraph[:300] + "..."
                            answer = f"Based on the book content: {paragraph} For more details, see: {retrieved_chunks[0]['url'] if retrieved_chunks else 'the book'}"
                            break
                    else:
                        first_part = clean_content[:200] + "..." if len(clean_content) > 200 else clean_content
                        answer = f"Based on the book content: {first_part} For more details, see: {retrieved_chunks[0]['url'] if retrieved_chunks else 'the book'}"
            else:
                answer = "I couldn't find relevant information in the knowledge base to answer your question."

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
