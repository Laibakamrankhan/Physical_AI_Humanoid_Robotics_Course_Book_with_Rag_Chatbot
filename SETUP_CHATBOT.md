# Setting up the Chatbot with Book Data

This guide will help you set up the RAG chatbot to work with the Physical AI & Humanoid Robotics book content.

## Prerequisites

1. **API Keys Required**:
   - **Cohere API Key**: For embeddings and language processing [Get it here](https://dashboard.cohere.com/api-keys)
   - **Qdrant URL**: For vector database (optional: you can run locally)
   - **OpenAI API Key**: For language model access (optional, if using OpenAI models)

2. **Running Services**:
   - Backend server on port 8000
   - Qdrant vector database (port 6333)
   - PostgreSQL database

## Step 1: Set up API Keys

Update your `.env` file with your actual API keys:

```env
# API Keys
COHERE_API_KEY=your_actual_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster-url.qdrant.tech:6333
QDRANT_API_KEY=your_qdrant_api_key_here
OPENAI_API_KEY=your_openai_api_key_here  # optional
NEON_DATABASE_URL=your_postgresql_connection_string_here
```

## Step 2: Alternative - Run Qdrant Locally (Optional)

If you don't have a Qdrant cloud account, you can run it locally:

```bash
docker run -p 6333:6333 qdrant/qdrant
```

Then update your `.env`:
```env
QDRANT_URL=http://localhost:6333
```

## Step 3: Start the Backend Server

```bash
cd backend
python -m uvicorn main:app --reload
```

## Step 4: Ingest the Book Documents

Once the backend is running and API keys are set, run the ingestion:

**On Windows:**
```bash
# Set your environment variables first
set COHERE_API_KEY=your_cohere_api_key
set QDRANT_URL=http://localhost:6333  # or your Qdrant URL
set QDRANT_API_KEY=your_qdrant_api_key

# Then run the ingestion
curl -X POST http://localhost:8000/api/v1/ingest \
     -H "Content-Type: application/json" \
     -d '{
       "force_reindex": true,
       "document_path": "1-physical-ai-robotics/"
     }'
```

**On Linux/Mac:**
```bash
# Set your environment variables first
export COHERE_API_KEY=your_cohere_api_key
export QDRANT_URL=your_qdrant_url
export QDRANT_API_KEY=your_qdrant_api_key

# Then run the ingestion
curl -X POST http://localhost:8000/api/v1/ingest \
     -H "Content-Type: application/json" \
     -d '{
       "force_reindex": true,
       "document_path": "1-physical-ai-robotics/"
     }'
```

Or use the provided scripts:
- `ingest_documents.bat` (Windows)
- `ingest_documents.sh` (Linux/Mac)

## Step 5: Verify Setup

After ingestion completes, you can test the chat functionality:

1. Start the frontend: `npx docusaurus start`
2. Go to the website (http://localhost:3000/Physical_AI_Humanoid_Robotics_Book/)
3. Use the chat widget to ask questions about the book content

## Troubleshooting

- **"COHERE_API_KEY environment variable is required"**: Make sure you've set the COHERE_API_KEY environment variable
- **Connection errors**: Verify that all required services are running
- **Document not found errors**: Make sure the document path is correct and files exist
- **Rate limit errors**: You may need to adjust the rate limiting in your .env file

## Notes

- The ingestion process may take several minutes depending on the document size
- Once ingested, the chatbot will be able to answer questions based on the book content
- The vector database stores document embeddings for semantic search
- You only need to run ingestion once, or when you update the book content