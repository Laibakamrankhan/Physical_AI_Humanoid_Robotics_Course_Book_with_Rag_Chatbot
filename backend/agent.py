import os
import json
import logging
import time
import re
from typing import Dict, List
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Cohere LLM
from langchain_cohere import ChatCohere

# Your custom retriever
from retriving import RAGRetriever

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGAgent:
    def __init__(self):
        # Initialize Cohere LLM
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            logger.warning("COHERE_API_KEY not set! Agent will run in retrieval-only mode.")
            self.llm = None
        else:
            self.llm = ChatCohere(
                model="command-r",
                temperature=0.3,
                cohere_api_key=cohere_api_key
            )
            logger.info("RAG Agent initialized with Cohere LLM")

        # Initialize retriever
        self.retriever = RAGRetriever()
        logger.info("RAG Retriever initialized successfully")

    def retrieve_information(self, query: str) -> List[Dict]:
        """
        Retrieve information from the knowledge base
        """
        try:
            raw_response = self.retriever.retrieve(query_text=query, top_k=5, threshold=0.1)
            results = json.loads(raw_response)
            formatted = []
            for r in results.get('results', []):
                formatted.append({
                    "content": r["content"],
                    "url": r["url"],
                    "position": r["position"],
                    "similarity_score": r["similarity_score"]
                })
            return formatted
        except Exception as e:
            logger.error(f"Error retrieving information: {e}")
            return []

    def generate_answer(self, query: str, retrieved_chunks: List[Dict]) -> str:
        """
        Generate answer from retrieved chunks using Cohere or fallback
        """
        if not retrieved_chunks:
            return "I couldn't find relevant information in the knowledge base. Please ask about robotics concepts covered in the book."

        # Prepare context
        context_str = "\n\n".join([f"Source: {c['url']}\nContent: {c['content']}" for c in retrieved_chunks])

        if self.llm:
            # Cohere LLM generation
            prompt = f"""
You are an expert robotics assistant.
Answer the question using ONLY the context below.
Provide concise, clear, and human-readable answers. Cite sources.

Context:
{context_str}

Question:
{query}
"""
            try:
                response = self.llm.invoke(prompt)
                return response.content.strip()
            except Exception as e:
                logger.error(f"Cohere generation failed: {e}")
                return f"Based on the book content: {retrieved_chunks[0]['content'][:300]}... For more details, see: {retrieved_chunks[0]['url']}"
        else:
            # Fallback: simple retrieval-only
            top_chunk = retrieved_chunks[0]
            content = re.sub(r'^#+\s+', '', top_chunk['content'], flags=re.MULTILINE)
            return f"Based on the book content: {content[:300]}... For more details, see: {top_chunk['url']}"

    def query_agent(self, query_text: str) -> Dict:
        """
        Main entry for querying the agent
        """
        start_time = time.time()
        retrieved_chunks = self.retrieve_information(query_text)
        answer = self.generate_answer(query_text, retrieved_chunks)

        # Confidence based on similarity
        confidence = "low"
        if retrieved_chunks:
            avg_score = sum(c.get("similarity_score", 0.0) for c in retrieved_chunks) / len(retrieved_chunks)
            if avg_score >= 0.7:
                confidence = "high"
            elif avg_score >= 0.4:
                confidence = "medium"

        query_time_ms = (time.time() - start_time) * 1000

        return {
            "answer": answer,
            "sources": [c['url'] for c in retrieved_chunks if c['url']],
            "matched_chunks": retrieved_chunks,
            "query_time_ms": query_time_ms,
            "confidence": confidence
        }

# Convenience function
def query_agent(query_text: str) -> Dict:
    agent = RAGAgent()
    return agent.query_agent(query_text)

# Example main for testing
if __name__ == "__main__":
    agent = RAGAgent()
    test_queries = [
        "What is a digital twin in robotics?",
        "Explain humanoid robot sensors."
    ]
    for q in test_queries:
        response = agent.query_agent(q)
        print(f"Query: {q}")
        print(f"Answer: {response['answer']}")
        print(f"Sources: {response.get('sources', [])}")
        print(f"Confidence: {response.get('confidence')}")
        print("-"*50)
