import os
import json
import logging
from typing import Dict, List, Any
from dotenv import load_dotenv
import openai
from openai import OpenAI
import asyncio
import time

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
        json_response = retriever.retrieve(query_text=query, top_k=5, threshold=0.1)  # Lower threshold to get more results
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
        # Initialize OpenAI client if API key is available
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key and openai_api_key != "OPENAI_API_KEY":  # Check if it's a placeholder
            self.client = OpenAI(api_key=openai_api_key)
            logger.info("RAG Agent initialized with OpenAI API")
        else:
            self.client = None
            logger.info("RAG Agent initialized without OpenAI API (using local LLM or rule-based responses)")

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

            # Prepare the prompt with retrieved context
            if context_str:
                system_prompt = f"""You are a helpful assistant that answers questions based on retrieved documents.
                Use the following context to answer the user's question. Always cite your sources.

                Context:
                {context_str}"""
            else:
                system_prompt = "You are a helpful assistant. Answer the user's question to the best of your ability."

            # If OpenAI client is available, use it; otherwise generate response from context
            if self.client:
                # Call OpenAI API with the context
                response = self.client.chat.completions.create(
                    model=os.getenv("LLM_MODEL", "gpt-4-turbo"),
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": query_text}
                    ],
                    max_tokens=1000,
                    temperature=0.7
                )

                answer = response.choices[0].message.content
            else:
                # Generate a response based on the retrieved context without calling OpenAI
                # This is a simplified approach when OpenAI API is not available
                if context_str:
                    # Extract key information from context to form a concise response
                    # Take only the most relevant parts and summarize
                    relevant_chunks = retrieved_chunks[:2]  # Take top 2 most relevant chunks for conciseness
                    concise_parts = []

                    for chunk in relevant_chunks:
                        content = chunk['content']
                        # Clean the content by removing markdown headers and other formatting
                        import re
                        # Remove markdown headers (# ## ###)
                        clean_content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
                        # Remove other common markdown elements
                        clean_content = re.sub(r'\n\d+\.\d*\.?\s+', '\n', clean_content)  # Remove numbered sections
                        clean_content = re.sub(r'\n\s*-\s+', '\n• ', clean_content)  # Convert list items to bullet points

                        # Find the most relevant sentence containing the query terms
                        sentences = clean_content.split('.')
                        query_lower = query_text.lower()
                        relevant_sentences = []

                        for sentence in sentences:
                            # Skip sentences that look like headers or section titles
                            clean_sentence = sentence.strip()
                            if len(clean_sentence) < 10:  # Skip very short sentences
                                continue
                            if any(header_marker in clean_sentence for header_marker in ['#', '##', '###', 'Chapter', 'Section', 'Module']):
                                continue
                            if any(term in clean_sentence.lower() for term in query_lower.split()[:3]):  # Use first 3 query terms
                                relevant_sentences.append(clean_sentence)

                        if relevant_sentences:
                            # Take up to 2 most relevant sentences
                            selected = '. '.join(relevant_sentences[:2]) + '.'
                            if len(selected) > 250:  # Truncate if too long
                                selected = selected[:250] + "..."
                            concise_parts.append(selected.strip())

                    if concise_parts:
                        answer = "Based on the book content: " + " ".join(concise_parts[:2])  # Combine with spaces, limit to 2 parts
                        answer += f" For more details, see: {retrieved_chunks[0]['url'] if retrieved_chunks else 'the book'}"
                    else:
                        # Fallback: extract a concise summary from the most relevant chunk
                        most_relevant = retrieved_chunks[0]['content'] if retrieved_chunks else ""
                        import re
                        # Clean the content
                        clean_content = re.sub(r'^#+\s+', '', most_relevant, flags=re.MULTILINE)
                        clean_content = re.sub(r'\n\d+\.\d*\.?\s+', '\n', clean_content)

                        # Take the first meaningful paragraph that's not a header
                        paragraphs = [p.strip() for p in clean_content.split('\n') if p.strip()]
                        for paragraph in paragraphs:
                            if len(paragraph) > 50 and not any(header_marker in paragraph for header_marker in ['#', '##', '###', 'Chapter', 'Section', 'Module']):
                                if len(paragraph) > 300:
                                    paragraph = paragraph[:300] + "..."
                                answer = f"Based on the book content: {paragraph} For more details, see: {retrieved_chunks[0]['url'] if retrieved_chunks else 'the book'}"
                                break
                        else:
                            # If no good paragraph found, use a shorter version of the first part
                            first_part = clean_content[:200] + "..." if len(clean_content) > 200 else clean_content
                            answer = f"Based on the book content: {first_part} For more details, see: {retrieved_chunks[0]['url'] if retrieved_chunks else 'the book'}"
                else:
                    answer = "I couldn't find relevant information in the knowledge base to answer your question. Please try rephrasing or ask about robotics concepts covered in the book."

            # Extract sources from retrieved chunks
            sources = list(set([chunk['url'] for chunk in retrieved_chunks if chunk['url']]))

            # Calculate query time
            query_time_ms = (time.time() - start_time) * 1000

            response_data = {
                "answer": answer,
                "sources": sources,
                "matched_chunks": retrieved_chunks,
                "query_time_ms": query_time_ms,
                "confidence": self._calculate_confidence(retrieved_chunks)
            }

            logger.info(f"Query processed in {query_time_ms:.2f}ms")
            return response_data

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
        """
        Calculate confidence level based on similarity scores and number of matches
        """
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
    """
    Convenience function to query the RAG agent
    """
    agent = RAGAgent()
    return agent.query_agent(query_text)

def run_agent_sync(query_text: str) -> Dict:
    """
    Synchronous function to run the agent for direct usage
    """
    import asyncio

    async def run_async():
        agent = RAGAgent()
        return await agent._async_query_agent(query_text)

    # Check if there's already a running event loop
    try:
        loop = asyncio.get_running_loop()
        # If there's already a loop, run in a separate thread
        import concurrent.futures
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(asyncio.run, run_async())
            return future.result()
    except RuntimeError:
        # No running loop, safe to use asyncio.run
        return asyncio.run(run_async())

def main():
    """
    Main function to demonstrate the RAG agent functionality
    """
    logger.info("Initializing RAG Agent...")

    # Initialize the agent
    agent = RAGAgent()

    # Example queries to test the system
    test_queries = [
        "what is a digital twin in robotics?",
    ]

    print("RAG Agent - Testing Queries")
    print("=" * 50)

    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        print("-" * 30)

        # Process query through agent
        response = agent.query_agent(query)

        # Print formatted results
        print(f"Answer: {response['answer']}")

        if response.get('sources'):
            print(f"Sources: {len(response['sources'])} documents")
            for source in response['sources'][:3]:  # Show first 3 sources
                print(f"  - {source}")

        if response.get('matched_chunks'):
            print(f"Matched chunks: {len(response['matched_chunks'])}")
            for j, chunk in enumerate(response['matched_chunks'][:2], 1):  # Show first 2 chunks
                content_preview = chunk['content'][:100] + "..." if len(chunk['content']) > 100 else chunk['content']
                print(f"  Chunk {j}: {content_preview}")
                print(f"    Source: {chunk['url']}")
                print(f"    Score: {chunk['similarity_score']:.3f}")

        print(f"Query time: {response['query_time_ms']:.2f}ms")
        print(f"Confidence: {response.get('confidence', 'unknown')}")

        if i < len(test_queries):  # Don't sleep after the last query
            time.sleep(1)  # Small delay between queries

if __name__ == "__main__":
    main()