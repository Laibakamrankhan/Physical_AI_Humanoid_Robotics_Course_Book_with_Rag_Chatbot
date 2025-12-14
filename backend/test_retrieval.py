import os
from dotenv import load_dotenv
from retriving import RAGRetriever
import json

# Load environment variables
load_dotenv()

def test_retrieval():
    print("Testing RAG retrieval system...")

    # Initialize the retriever
    retriever = RAGRetriever()

    # Test query
    test_query = "digital twin"
    print(f"Query: {test_query}")

    # Perform retrieval
    result = retriever.retrieve(query_text=test_query, top_k=5, threshold=0.0)  # Use 0 threshold to get all results
    result_dict = json.loads(result)

    print(f"Results: {result_dict}")

    # Check if we got any results
    results = result_dict.get('results', [])
    print(f"Number of results: {len(results)}")

    if results:
        print("Sample results:")
        for i, result in enumerate(results[:2]):  # Show first 2 results
            print(f"  {i+1}. Content preview: {result['content'][:200]}...")
            print(f"     URL: {result['url']}")
            print(f"     Score: {result['similarity_score']}")
            print()
    else:
        print("No results found - checking if database is empty...")

        # Check if there are any points in the collection
        try:
            count = retriever.qdrant_client.count(
                collection_name=retriever.collection_name
            )
            print(f"Total points in collection: {count.count}")
        except Exception as e:
            print(f"Error checking collection: {e}")

if __name__ == "__main__":
    test_retrieval()